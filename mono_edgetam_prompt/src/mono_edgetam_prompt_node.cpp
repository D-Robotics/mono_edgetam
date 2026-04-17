// Copyright (c) 2026，D-Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cstring>
#include <sstream>
#include <time.h>

#include "hobot_cv/hobotcv_imgproc.h"
#include "rclcpp/rclcpp.hpp"

#include "include/mono_edgetam_prompt_node.h"

// 时间格式转换
builtin_interfaces::msg::Time ConvertToRosTime(
    const struct timespec &time_spec) {
  builtin_interfaces::msg::Time stamp;
  stamp.set__sec(time_spec.tv_sec);
  stamp.set__nanosec(time_spec.tv_nsec);
  return stamp;
}

// 根据起始时间计算耗时
int CalTimeMsDuration(const builtin_interfaces::msg::Time &start,
                      const builtin_interfaces::msg::Time &end) {
  return (end.sec - start.sec) * 1000 + end.nanosec / 1000 / 1000 -
         start.nanosec / 1000 / 1000;
}

int ResizeNV12Img(const char *in_img_data,
                  const int &in_img_height,
                  const int &in_img_width,
                  const int &scaled_img_height,
                  const int &scaled_img_width,
                  cv::Mat &out_img) {
  cv::Mat src(
      in_img_height * 3 / 2, in_img_width, CV_8UC1, (void *)(in_img_data));

  int ret = hobot_cv::hobotcv_resize(
      src, in_img_height, in_img_width, out_img, scaled_img_height, scaled_img_width);

  return ret;
}

MonoEdgetamPromptNode::MonoEdgetamPromptNode(const std::string& node_name,
                                           const NodeOptions& options)
    : DnnNode(node_name, options) {
  this->declare_parameter<int>("feed_type", feed_type_);
  this->declare_parameter<int>("is_sync_mode", is_sync_mode_);
  this->declare_parameter<int>("is_shared_mem_sub", is_shared_mem_sub_);
  this->declare_parameter<int>("prompt_mode", prompt_mode_);
  this->declare_parameter<int>("dump_render_img", dump_render_img_);
  this->declare_parameter<std::string>("model_file_name", model_file_name_);
  this->declare_parameter<std::string>("dump_mem_feat_path", dump_mem_feat_path_);
  this->declare_parameter<std::string>("dump_mem_ptr_path", dump_mem_ptr_path_);
  this->declare_parameter<std::string>("image", image_file_);
  this->declare_parameter<std::string>("ros_img_topic_name", ros_img_topic_name_);
  this->declare_parameter<std::string>("ai_msg_pub_topic_name", ai_msg_pub_topic_name_);
  this->declare_parameter<std::string>("ai_msg_sub_topic_name", ai_msg_sub_topic_name_);

  this->get_parameter<int>("feed_type", feed_type_);
  this->get_parameter<int>("is_sync_mode", is_sync_mode_);
  this->get_parameter<int>("is_shared_mem_sub", is_shared_mem_sub_);
  this->get_parameter<int>("prompt_mode", prompt_mode_);
  this->get_parameter<int>("dump_render_img", dump_render_img_);
  this->get_parameter<std::string>("model_file_name", model_file_name_);
  this->get_parameter<std::string>("dump_mem_feat_path", dump_mem_feat_path_);
  this->get_parameter<std::string>("dump_mem_ptr_path", dump_mem_ptr_path_);
  this->get_parameter<std::string>("image", image_file_);
  this->get_parameter<std::string>("ros_img_topic_name", ros_img_topic_name_);
  this->get_parameter<std::string>("ai_msg_pub_topic_name", ai_msg_pub_topic_name_);
  this->get_parameter<std::string>("ai_msg_sub_topic_name", ai_msg_sub_topic_name_);

#ifdef SHARED_MEM_ENABLED
  this->declare_parameter<std::string>("sharedmem_img_topic_name",
                                       sharedmem_img_topic_name_);
  this->get_parameter<std::string>("sharedmem_img_topic_name",
                                   sharedmem_img_topic_name_);
#endif

  std::stringstream ss;
  ss << "mono_edgetam_prompt params:"
     << "\n feed_type(0:local, 1:sub): " << feed_type_
     << "\n image: " << image_file_
     << "\n is_sync_mode: " << is_sync_mode_
     << "\n is_shared_mem_sub: " << is_shared_mem_sub_
     << "\n prompt_mode: " << prompt_mode_
     << "\n ros_img_topic_name: " << ros_img_topic_name_
     << "\n ai_msg_pub_topic_name: " << ai_msg_pub_topic_name_
     << "\n ai_msg_sub_topic_name: " << ai_msg_sub_topic_name_
     << "\n model_file_name: " << model_file_name_
     << "\n dump_mem_feat_path: " << dump_mem_feat_path_
     << "\n dump_mem_ptr_path: " << dump_mem_ptr_path_
     << "\n dump_render_img: " << dump_render_img_;
  RCLCPP_WARN(this->get_logger(), "%s", ss.str().c_str());

  if (Init() != 0) {
    RCLCPP_ERROR(this->get_logger(), "Init failed!");
  }

  if (model_name_.empty()) {
    if (!GetModel()) {
      RCLCPP_ERROR(this->get_logger(), "Get model fail.");
    } else {
      model_name_ = GetModel()->GetName();
      RCLCPP_WARN(this->get_logger(), "Model name: %s", model_name_.c_str());
    }
  }

  if (prompt_mode_ == 0) {
    point_coords_ = {320, 0, 533, 758};
    point_labels_ = {2, 3};
  } else if (prompt_mode_ == 1) {
    point_coords_ = {224, 663, 266, 417};
    point_labels_ = {1, 1};
  }

  if (feed_type_ == 0) {
    FeedFromLocal();
  } else {

    ai_msg_publisher_ = this->create_publisher<ai_msgs::msg::PerceptionTargets>(
      ai_msg_pub_topic_name_, 10);
    RCLCPP_WARN(this->get_logger(),
            "Create ai msg publisher with topic_name: %s",
            ai_msg_pub_topic_name_.c_str());

    ai_msg_subscription_ =
        this->create_subscription<ai_msgs::msg::PerceptionTargets>(
            ai_msg_sub_topic_name_,
            10,
            std::bind(
                &MonoEdgetamPromptNode::AiMsgProcess, this, std::placeholders::_1));

#ifdef SHARED_MEM_ENABLED
    if (is_shared_mem_sub_) {
      sharedmem_img_subscription_ =
          this->create_subscription<hbm_img_msgs::msg::HbmMsg1080P>(
              sharedmem_img_topic_name_,
              rclcpp::SensorDataQoS(),
              std::bind(&MonoEdgetamPromptNode::SharedMemImgProcess, this,
                        std::placeholders::_1));
    } else
#endif
    {
      ros_img_subscription_ =
          this->create_subscription<sensor_msgs::msg::Image>(
              ros_img_topic_name_,
              10,
              std::bind(&MonoEdgetamPromptNode::RosImgProcess, this,
                        std::placeholders::_1));
    }
  }
}

MonoEdgetamPromptNode::~MonoEdgetamPromptNode() = default;

int MonoEdgetamPromptNode::SetNodePara() {
  if (!dnn_node_para_ptr_) {
    return -1;
  }
  dnn_node_para_ptr_->model_file = model_file_name_;
  dnn_node_para_ptr_->model_name = model_name_;
  dnn_node_para_ptr_->model_task_type = model_task_type_;
  dnn_node_para_ptr_->task_num = 4;
  return 0;
}

int MonoEdgetamPromptNode::PostProcess(
    const std::shared_ptr<DnnNodeOutput>& node_output) {
  if (!rclcpp::ok()) {
    return 0;
  }

  auto parser_output = std::dynamic_pointer_cast<EdgeTamOutput>(node_output);
  auto prompt_parser = std::make_shared<PromptOutputParser>();

  if (!parser_output) {
    RCLCPP_ERROR(this->get_logger(), "Invalid node output");
    return -1;
  }

  // 1. 获取后处理开始时间
  struct timespec time_now = {0, 0};
  clock_gettime(CLOCK_REALTIME, &time_now);

  // 2. 模型后处理解析
  auto det_result = std::make_shared<DnnParserResult>();

  prompt_parser->Parse(det_result, parser_output->output_tensors[0], parser_output->img_h,
                         parser_output->img_w);

  // 如果开启了渲染，本地渲染并存储图片
  if (dump_render_img_ && feed_type_ == 0) {
    const std::string out_render = "render.jpeg";
    prompt_parser->RenderSeg(image_file_, det_result->perception.seg, out_render);
    RCLCPP_WARN(this->get_logger(), "Save Render Image: %s", out_render.c_str());
  }

  parser_output->output_tensors[1]->CACHE_INVALIDATE();
  float* float_data1 = parser_output->output_tensors[1]->GetTensorData<float>();
  WriteBinaryFile(dump_mem_feat_path_, float_data1, 1 * 512 * 64);

  parser_output->output_tensors[2]->CACHE_INVALIDATE();
  float* float_data2 = parser_output->output_tensors[2]->GetTensorData<float>();
  WriteBinaryFile(dump_mem_ptr_path_, float_data2, 1 * 256);

  if (feed_type_ == 0) {
    return 0;
  }

  // 3. 创建用于发布的AI消息
  if (!ai_msg_publisher_) {
    RCLCPP_ERROR(this->get_logger(), "Invalid msg publisher");
    return -1;
  }
  ai_msgs::msg::PerceptionTargets::UniquePtr pub_data(
      new ai_msgs::msg::PerceptionTargets());
  // 3.1 发布分割AI消息
  auto &seg = det_result->perception.seg;
  if (seg.height != 0 && seg.width != 0) {
    ai_msgs::msg::Capture capture;
    capture.features.swap(seg.data);
    capture.img.height = seg.valid_h;
    capture.img.width = seg.valid_w;
    capture.img.step = 1;

    RCLCPP_INFO(this->get_logger(),
                "features size: %d, width: %d, height: %d, num_classes: %d, step: %d",
                capture.features.size(),
                capture.img.width,
                capture.img.height,
                seg.num_classes,
                capture.img.step);

    ai_msgs::msg::Target target;
    target.set__type("segmentation");

    ai_msgs::msg::Attribute attribute;
    attribute.set__type("segmentation_label_count");
    attribute.set__value(seg.num_classes);
    target.attributes.emplace_back(std::move(attribute));

    target.captures.emplace_back(std::move(capture));
    pub_data->targets.emplace_back(std::move(target));
  }

  if (point_labels_[0] == 1 || point_labels_[1] == 1) {
    for (int i = 0; i < point_labels_.size(); ++i) {
      if (point_labels_[i] != 1) {
        continue;
      }
      ai_msgs::msg::Roi roi;
      roi.set__type("target");

      u_int32_t x_offset = static_cast<u_int32_t>(point_coords_[i * 2] * parser_output->img_w / model_input_width_);
      u_int32_t y_offset = static_cast<u_int32_t>(point_coords_[i * 2 + 1] * parser_output->img_h / model_input_height_);

      roi.rect.set__x_offset(x_offset);
      roi.rect.set__y_offset(y_offset);
      roi.rect.set__width(0);
      roi.rect.set__height(0);
      roi.set__confidence(1.0);

      ai_msgs::msg::Target target;
      target.set__type("target");
      target.rois.emplace_back(roi);
      pub_data->targets.emplace_back(std::move(target));
    }
  } else if (point_labels_[0] == 2 && point_labels_[1] == 3) { 
    ai_msgs::msg::Roi roi;
    roi.set__type("target");

    u_int32_t x_offset = static_cast<u_int32_t>(point_coords_[0] * parser_output->img_w / model_input_width_);
    u_int32_t y_offset = static_cast<u_int32_t>(point_coords_[1] * parser_output->img_h / model_input_height_);
    u_int32_t width = static_cast<u_int32_t>(point_coords_[2] * parser_output->img_w / model_input_width_) - x_offset;
    u_int32_t height = static_cast<u_int32_t>(point_coords_[3] * parser_output->img_h / model_input_height_) - y_offset;

    roi.rect.set__x_offset(x_offset);
    roi.rect.set__y_offset(y_offset);
    roi.rect.set__width(width);
    roi.rect.set__height(height);
    roi.set__confidence(1.0);

    ai_msgs::msg::Target target;
    target.set__type("target");
    target.rois.emplace_back(roi);
    pub_data->targets.emplace_back(std::move(target));
  }

  pub_data->header.set__stamp(parser_output->msg_header->stamp);
  pub_data->header.set__frame_id(parser_output->msg_header->frame_id);

  // 填充perf性能统计信息
  // 前处理统计
  pub_data->perfs.push_back(parser_output->perf_preprocess);

  // dnn node有输出统计信息
  if (node_output->rt_stat) {

    // 推理统计
    ai_msgs::msg::Perf perf;
    perf.set__type(model_name_ + "_predict_infer");
    perf.stamp_start =
        ConvertToRosTime(node_output->rt_stat->infer_timespec_start);
    perf.stamp_end = ConvertToRosTime(node_output->rt_stat->infer_timespec_end);
    perf.set__time_ms_duration(node_output->rt_stat->infer_time_ms);
    pub_data->perfs.push_back(perf);

    perf.set__type(model_name_ + "_predict_parse");
    perf.stamp_start =
        ConvertToRosTime(node_output->rt_stat->parse_timespec_start);
    perf.stamp_end = ConvertToRosTime(node_output->rt_stat->parse_timespec_end);
    perf.set__time_ms_duration(node_output->rt_stat->parse_time_ms);
    pub_data->perfs.push_back(perf);

    // 后处理统计
    ai_msgs::msg::Perf perf_postprocess;
    perf_postprocess.set__type(model_name_ + "_postprocess");
    perf_postprocess.stamp_start = ConvertToRosTime(time_now);
    clock_gettime(CLOCK_REALTIME, &time_now);
    perf_postprocess.stamp_end = ConvertToRosTime(time_now);
    perf_postprocess.set__time_ms_duration(CalTimeMsDuration(
        perf_postprocess.stamp_start, perf_postprocess.stamp_end));
    pub_data->perfs.emplace_back(perf_postprocess);

    // 从发布图像到发布AI结果的延迟
    ai_msgs::msg::Perf perf_pipeline;
    perf_pipeline.set__type(model_name_ + "_pipeline");
    perf_pipeline.set__stamp_start(pub_data->header.stamp);
    perf_pipeline.set__stamp_end(perf_postprocess.stamp_end);
    perf_pipeline.set__time_ms_duration(
        CalTimeMsDuration(perf_pipeline.stamp_start, perf_pipeline.stamp_end));
    pub_data->perfs.push_back(perf_pipeline);

    if (parser_output) {
      // Output time delay info
      RCLCPP_DEBUG_STREAM(this->get_logger(),
        "frame_id: " << parser_output->msg_header->frame_id
        << ", stamp: " << parser_output->msg_header->stamp.sec << "."
        << parser_output->msg_header->stamp.nanosec
        << ", preprocess time ms: " << static_cast<int>(parser_output->perf_preprocess.time_ms_duration)
        << ", infer time ms: " << node_output->rt_stat->infer_time_ms
        << ", post process time ms: " << static_cast<int>(perf_postprocess.time_ms_duration)
        << ", pipeline time ms: " << static_cast<int>(perf_pipeline.time_ms_duration)
        << ", infer_timespec_start: " << std::to_string(node_output->rt_stat->infer_timespec_start.tv_sec) << "." << std::to_string(node_output->rt_stat->infer_timespec_start.tv_nsec)
        << ", infer_timespec_end: " << std::to_string(node_output->rt_stat->infer_timespec_end.tv_sec) << "." << std::to_string(node_output->rt_stat->infer_timespec_end.tv_nsec)
        << ", parse_timespec_start: " << std::to_string(node_output->rt_stat->parse_timespec_start.tv_sec) << "." << std::to_string(node_output->rt_stat->parse_timespec_start.tv_nsec)
        << ", parse_timespec_end: " << std::to_string(node_output->rt_stat->parse_timespec_end.tv_sec) << "." << std::to_string(node_output->rt_stat->parse_timespec_end.tv_nsec)
      );
    }

    // 推理输出帧率统计
    pub_data->set__fps(round(node_output->rt_stat->output_fps));

    // 如果当前帧有更新统计信息，输出统计信息
    if (node_output->rt_stat->fps_updated) {
      RCLCPP_WARN(this->get_logger(),
                  "Sub img fps: %.2f, Smart fps: %.2f, "
                  "pre process time ms: %d, infer time ms: %d, "
                  "post process time ms: %d, "
                  "pipeline time ms: %d",
                  node_output->rt_stat->input_fps,
                  node_output->rt_stat->output_fps,
                  static_cast<int>(parser_output->perf_preprocess.time_ms_duration),
                  node_output->rt_stat->infer_time_ms,
                  static_cast<int>(perf_postprocess.time_ms_duration),
                  static_cast<int>(perf_pipeline.time_ms_duration)
                );
    }
  }

  // 发布AI消息
  ai_msg_publisher_->publish(std::move(pub_data));

  return 0;
}

void MonoEdgetamPromptNode::RosImgProcess(
    const sensor_msgs::msg::Image::ConstSharedPtr img_msg) {
  if (!img_msg || !rclcpp::ok()) {
    return;
  }

  struct timespec time_now = {0, 0};
  clock_gettime(CLOCK_REALTIME, &time_now);

  std::stringstream ss;
  ss << "Recved img encoding: " << img_msg->encoding
     << ", h: " << img_msg->height << ", w: " << img_msg->width
     << ", step: " << img_msg->step
     << ", frame_id: " << img_msg->header.frame_id
     << ", stamp: " << img_msg->header.stamp.sec << "_"
     << img_msg->header.stamp.nanosec
     << ", data size: " << img_msg->data.size();
  RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());

  image_input_width_ = img_msg->width;
  image_input_height_ = img_msg->height;

  // 1. 将图片处理成模型输入数据类型DNNInput
  // 使用图片生成pym，NV12PyramidInput为DNNInput的子类
  std::pair<std::shared_ptr<DNNTensor>, std::shared_ptr<DNNTensor>> nv12_tensor = std::pair<std::shared_ptr<DNNTensor>, std::shared_ptr<DNNTensor>>(nullptr, nullptr);
  hbDNNTensorProperties tensor_y_properties;
  GetModel()->GetInputTensorProperties(tensor_y_properties, 0);
  hbDNNTensorProperties tensor_uv_properties;
  GetModel()->GetInputTensorProperties(tensor_uv_properties, 1);
  if ("nv12" == img_msg->encoding) {  // nv12格式使用hobotcv resize
    if (img_msg->height != static_cast<uint32_t>(model_input_height_) ||
        img_msg->width != static_cast<uint32_t>(model_input_width_)) {
      // 需要做resize处理
      cv::Mat out_img;
      if (ResizeNV12Img(reinterpret_cast<const char *>(img_msg->data.data()),
                        img_msg->height,
                        img_msg->width,
                        model_input_height_,
                        model_input_width_,
                        out_img) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Resize nv12 img fail!");
        return;
      }
      nv12_tensor = ImageProc::GetNV12TensorsFromNV12Img(
          reinterpret_cast<const char *>(out_img.data),
          tensor_y_properties,
          tensor_uv_properties,
          model_input_height_,
          model_input_width_);
    } else {  //不需要进行resize
      nv12_tensor = 
        ImageProc::GetNV12TensorsFromNV12Img(
          reinterpret_cast<const char*>(img_msg->data.data()),
          tensor_y_properties,
          tensor_uv_properties,
          model_input_height_,
          model_input_width_);
    }
  } else {
    RCLCPP_ERROR(this->get_logger(),
                "Unsupported img encoding: %s",
                img_msg->encoding);
  }

  // 2.准备模型输入
  hbDNNTensorProperties tensor_point_coords_properties;
  GetModel()->GetInputTensorProperties(tensor_point_coords_properties, 2);
  hbDNNTensorProperties tensor_point_labels_properties;
  GetModel()->GetInputTensorProperties(tensor_point_labels_properties, 3);

  std::shared_ptr<DNNTensor> tensor_point_coords = GetMemoryTensor(point_coords_, tensor_point_coords_properties);
  std::shared_ptr<DNNTensor> tensor_point_labels = GetMemoryTensor(point_labels_, tensor_point_labels_properties);

  std::vector<std::shared_ptr<DNNTensor>> inputs;
  inputs.push_back(nv12_tensor.first);
  inputs.push_back(nv12_tensor.second);
  inputs.push_back(tensor_point_coords);
  inputs.push_back(tensor_point_labels);

  // 3. 创建推理输出数据
  auto dnn_output = std::make_shared<EdgeTamOutput>();
  // 将图片消息的header填充到输出数据中，用于表示推理输出对应的输入信息
  dnn_output->msg_header = std::make_shared<std_msgs::msg::Header>();
  dnn_output->msg_header->set__frame_id(img_msg->header.frame_id);
  dnn_output->msg_header->set__stamp(img_msg->header.stamp);
  // 将当前的时间戳填充到输出数据中，用于计算perf
  dnn_output->perf_preprocess.stamp_start.sec = time_now.tv_sec;
  dnn_output->perf_preprocess.stamp_start.nanosec = time_now.tv_nsec;
  dnn_output->perf_preprocess.set__type(model_name_ + "_preprocess");
  clock_gettime(CLOCK_REALTIME, &time_now);
  dnn_output->perf_preprocess.stamp_end.sec = time_now.tv_sec;
  dnn_output->perf_preprocess.stamp_end.nanosec = time_now.tv_nsec;
  dnn_output->img_h = img_msg->height;
  dnn_output->img_w = img_msg->width;

  // 4. 开始预测
  int ret = Run(inputs, dnn_output, is_sync_mode_ == 1 ? true : false);

  return;
}

#ifdef SHARED_MEM_ENABLED
void MonoEdgetamPromptNode::SharedMemImgProcess(
    const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr img_msg) {
  if (!img_msg || !rclcpp::ok()) {
    return;
  }

  struct timespec time_now = {0, 0};
  clock_gettime(CLOCK_REALTIME, &time_now);

  std::stringstream ss;
  ss << "Recved img encoding: "
     << std::string(reinterpret_cast<const char*>(img_msg->encoding.data()))
     << ", h: " << img_msg->height << ", w: " << img_msg->width
     << ", step: " << img_msg->step << ", index: " << img_msg->index
     << ", stamp: " << img_msg->time_stamp.sec << "_"
     << img_msg->time_stamp.nanosec << ", data size: " << img_msg->data_size;
  RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());

  image_input_width_ = img_msg->width;
  image_input_height_ = img_msg->height;

  // 1. 将图片处理成模型输入数据类型 DNNInput
  // 使用图片生成 pym，NV12PyramidInput 为 DNNInput 的子类
  std::pair<std::shared_ptr<DNNTensor>, std::shared_ptr<DNNTensor>> nv12_tensor = std::pair<std::shared_ptr<DNNTensor>, std::shared_ptr<DNNTensor>>(nullptr, nullptr);
  hbDNNTensorProperties tensor_y_properties;
  GetModel()->GetInputTensorProperties(tensor_y_properties, 0);
  hbDNNTensorProperties tensor_uv_properties;
  GetModel()->GetInputTensorProperties(tensor_uv_properties, 1);
  if ("nv12" ==
      std::string(reinterpret_cast<const char*>(img_msg->encoding.data()))) {
    if (img_msg->height != static_cast<uint32_t>(model_input_height_) ||
        img_msg->width != static_cast<uint32_t>(model_input_width_)) {
      cv::Mat out_img;
      if (ResizeNV12Img(reinterpret_cast<const char *>(img_msg->data.data()),
                        img_msg->height,
                        img_msg->width,
                        model_input_height_,
                        model_input_width_,
                        out_img) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Resize nv12 img fail!");
        return;
      }
      nv12_tensor = ImageProc::GetNV12TensorsFromNV12Img(
          reinterpret_cast<const char *>(out_img.data),
          tensor_y_properties,
          tensor_uv_properties,
          model_input_height_,
          model_input_width_);
    } else {
      nv12_tensor = 
        ImageProc::GetNV12TensorsFromNV12Img(
          reinterpret_cast<const char*>(img_msg->data.data()),
          tensor_y_properties, 
          tensor_uv_properties,
          model_input_height_,
          model_input_width_);
    }
  } else {
    RCLCPP_ERROR(this->get_logger(),
                "Unsupported img encoding: %s",
                img_msg->encoding);
  }

  // 2.准备模型输入
  hbDNNTensorProperties tensor_point_coords_properties;
  GetModel()->GetInputTensorProperties(tensor_point_coords_properties, 2);
  hbDNNTensorProperties tensor_point_labels_properties;
  GetModel()->GetInputTensorProperties(tensor_point_labels_properties, 3);

  std::shared_ptr<DNNTensor> tensor_point_coords = GetMemoryTensor(point_coords_, tensor_point_coords_properties);
  std::shared_ptr<DNNTensor> tensor_point_labels = GetMemoryTensor(point_labels_, tensor_point_labels_properties);

  std::vector<std::shared_ptr<DNNTensor>> inputs;
  inputs.push_back(nv12_tensor.first);
  inputs.push_back(nv12_tensor.second);
  inputs.push_back(tensor_point_coords);
  inputs.push_back(tensor_point_labels);

  // 3. 创建推理输出数据
  auto dnn_output = std::make_shared<EdgeTamOutput>();
  // 将图片消息的header填充到输出数据中，用于表示推理输出对应的输入信息
  dnn_output->msg_header = std::make_shared<std_msgs::msg::Header>();
  dnn_output->msg_header->set__frame_id(std::to_string(img_msg->index));
  dnn_output->msg_header->set__stamp(img_msg->time_stamp);
  // 将当前的时间戳填充到输出数据中，用于计算perf
  dnn_output->perf_preprocess.stamp_start.sec = time_now.tv_sec;
  dnn_output->perf_preprocess.stamp_start.nanosec = time_now.tv_nsec;
  dnn_output->perf_preprocess.set__type(model_name_ + "_preprocess");
  clock_gettime(CLOCK_REALTIME, &time_now);
  dnn_output->perf_preprocess.stamp_end.sec = time_now.tv_sec;
  dnn_output->perf_preprocess.stamp_end.nanosec = time_now.tv_nsec;
  dnn_output->img_h = img_msg->height;
  dnn_output->img_w = img_msg->width;

  // 4. 开始预测
  int ret = Run(inputs, dnn_output, is_sync_mode_ == 1 ? true : false);

  return;
}
#endif

int MonoEdgetamPromptNode::FeedFromLocal() {

  struct timespec time_now = {0, 0};
  clock_gettime(CLOCK_REALTIME, &time_now);

  // 1. 将图片处理成模型输入数据类型DNNTensor
  hbDNNTensorProperties tensor_y_properties;
  GetModel()->GetInputTensorProperties(tensor_y_properties, 0);
  hbDNNTensorProperties tensor_uv_properties;
  GetModel()->GetInputTensorProperties(tensor_uv_properties, 1);
  hbDNNTensorProperties tensor_point_coords_properties;
  GetModel()->GetInputTensorProperties(tensor_point_coords_properties, 2);
  hbDNNTensorProperties tensor_point_labels_properties;
  GetModel()->GetInputTensorProperties(tensor_point_labels_properties, 3);
  
  std::pair<std::shared_ptr<DNNTensor>, std::shared_ptr<DNNTensor>> nv12_tensor = 
    ImageProc::GetNV12TensorFromNV12(image_file_, tensor_y_properties, tensor_uv_properties, model_input_height_, model_input_width_);
  cv::Mat bgr_img = cv::imread(image_file_);
  int img_h = bgr_img.rows;
  int img_w = bgr_img.cols;
  std::shared_ptr<DNNTensor> tensor_point_coords = GetMemoryTensor(point_coords_, tensor_point_coords_properties);
  std::shared_ptr<DNNTensor> tensor_point_labels = GetMemoryTensor(point_labels_, tensor_point_labels_properties);

  // 2. 使用pyramid创建DNNInput对象inputs
  // inputs将会作为模型的输入通过RunInferTask接口传入
  std::vector<std::shared_ptr<DNNTensor>> inputs;
  inputs.push_back(nv12_tensor.first);
  inputs.push_back(nv12_tensor.second);
  inputs.push_back(tensor_point_coords);
  inputs.push_back(tensor_point_labels);

  auto dnn_output = std::make_shared<EdgeTamOutput>();
  // 将当前的时间戳填充到输出数据中，用于计算perf
  dnn_output->perf_preprocess.stamp_start.sec = time_now.tv_sec;
  dnn_output->perf_preprocess.stamp_start.nanosec = time_now.tv_nsec;
  dnn_output->perf_preprocess.set__type(model_name_ + "_preprocess");
  clock_gettime(CLOCK_REALTIME, &time_now);
  dnn_output->perf_preprocess.stamp_end.sec = time_now.tv_sec;
  dnn_output->perf_preprocess.stamp_end.nanosec = time_now.tv_nsec;
  dnn_output->img_h = img_h;
  dnn_output->img_w = img_w;

  uint32_t ret = 0;
  // 3. 开始预测
  ret = Run(inputs, dnn_output, is_sync_mode_ == 1 ? true : false);

  // 4. 处理预测结果，如渲染到图片或者发布预测结果
  if (ret != 0) {
    return -1;
  }

  return 0;
}

template <typename T>
std::shared_ptr<DNNTensor> MonoEdgetamPromptNode::GetMemoryTensor(
    const std::vector<T>& embeddings,
    hbDNNTensorProperties &tensor_properties) {

  auto *mem = new hbUCPSysMem;
  hbUCPMallocCached(mem, tensor_properties.alignedByteSize, 0);
  //内存初始化
  memset(mem->virAddr, 0, tensor_properties.alignedByteSize);

  auto *src = reinterpret_cast<const uint8_t *>(embeddings.data());

  std::memcpy(mem->virAddr, src, embeddings.size() * sizeof(T));

  hbUCPMemFlush(mem, HB_SYS_MEM_CACHE_CLEAN);
  auto input_tensor = new DNNTensor;

  input_tensor->properties = tensor_properties;
  input_tensor->sysMem.virAddr = reinterpret_cast<void *>(mem->virAddr);
  input_tensor->sysMem.phyAddr = mem->phyAddr;
  input_tensor->sysMem.memSize = tensor_properties.alignedByteSize;
  return std::shared_ptr<DNNTensor>(
      input_tensor, [mem](DNNTensor *input_tensor) {
        // Release memory after deletion
        hbUCPFree(mem);
        delete mem;
        delete input_tensor;
      });
}

void MonoEdgetamPromptNode::AiMsgProcess(
    const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg) {
  if (!msg || !rclcpp::ok()) {
    return;
  }

  if (image_input_width_ == 0 || image_input_height_ == 0) {
    return;
  }

  if (prompt_mode_ == 0) {
    float max_confidence = -1.0;
    u_int32_t x_offset = 0;
    u_int32_t y_offset = 0;
    uint32_t height = 0;
    uint32_t width = 0;

    for (const auto &target : msg->targets) {
      for (const auto &roi : target.rois) {
        if (roi.confidence > max_confidence) {
          max_confidence = roi.confidence;
          x_offset = roi.rect.x_offset;
          y_offset = roi.rect.y_offset;
          height = roi.rect.height;
          width = roi.rect.width;
        }
      }
    }
    point_coords_[0] = x_offset;
    point_coords_[1] = y_offset;
    point_coords_[2] = x_offset + width;
    point_coords_[3] = y_offset + height;
    point_labels_[0] = 2;
    point_labels_[1] = 3;
  } else if (prompt_mode_ == 1) {
    point_labels_[0] = -1;
    point_labels_[1] = -1;
    for (const auto &target : msg->targets) {
      for (int i = 0; i < target.rois.size(); ++i) {
          auto roi = target.rois[i];
          point_coords_[2 * i] = roi.rect.x_offset;
          point_coords_[2 * i + 1] = roi.rect.y_offset;
          point_labels_[i] = 1;
      }
    }
  }

  point_coords_[0] = point_coords_[0] * model_input_width_ / image_input_width_;
  point_coords_[1] = point_coords_[1] * model_input_height_ / image_input_height_;
  point_coords_[2] = point_coords_[2] * model_input_width_ / image_input_width_;
  point_coords_[3] = point_coords_[3] * model_input_height_ / image_input_height_;
}