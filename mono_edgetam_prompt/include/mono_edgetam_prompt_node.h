// Copyright (c) 2026，D-Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef MONO_EDGETAM_TRACK_NODE_H_
#define MONO_EDGETAM_TRACK_NODE_H_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "ai_msgs/msg/capture_targets.hpp"
#include "ai_msgs/msg/perception_targets.hpp"
#include "ai_msgs/msg/perf.hpp"
#include "dnn_node/dnn_node.h"
#include "dnn_node/dnn_node_data.h"
#include "dnn_node/util/output_parser/perception_common.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

#ifdef SHARED_MEM_ENABLED
#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"
#endif

#include "include/image_proc.h"
#include "include/prompt_output_parser.h"

using rclcpp::NodeOptions;
using hobot::dnn_node::DnnNode;
using hobot::dnn_node::DnnNodeOutput;
using hobot::dnn_node::DNNTensor;
using hobot::dnn_node::ModelTaskType;
using hobot::dnn_node::output_parser::DnnParserResult;
using hobot::dnn_node::output_parser::Perception;

using ai_msgs::msg::PerceptionTargets;

struct EdgeTamOutput : public DnnNodeOutput {
  std::shared_ptr<std_msgs::msg::Header> msg_header = nullptr;
  ai_msgs::msg::Perf perf_preprocess;
  int img_h = 0;
  int img_w = 0;
};

class MonoEdgetamPromptNode : public DnnNode {
 public:
  MonoEdgetamPromptNode(const std::string& node_name,
                       const NodeOptions& options = NodeOptions());
  ~MonoEdgetamPromptNode() override;

 protected:
  int SetNodePara() override;
  int PostProcess(const std::shared_ptr<DnnNodeOutput>& outputs) override;

 private:
  int feed_type_ = 0;
  int is_sync_mode_ = 0;
  int is_shared_mem_sub_ = 0;
  int prompt_mode_ = 0;
  int dump_render_img_ = 0;

  std::string model_file_name_ = "model_prompt_to_memory_points.hbm";
  std::string model_name_;
  ModelTaskType model_task_type_ = ModelTaskType::ModelInferType;

  std::string image_file_ = "bedroom/00000.jpg";

  std::string dump_mem_feat_path_ = "cond_maskmem_features.bin";
  std::string dump_mem_ptr_path_ = "cond_obj_ptr.bin";

  int model_input_width_ = 1024;
  int model_input_height_ = 1024;

  int image_input_width_ = 0;
  int image_input_height_ = 0;

  std::vector<std::shared_ptr<DNNTensor>> input_tensors_;
  
  std::vector<int> point_coords_ = {320, 0, 533, 758};
  std::vector<int> point_labels_ = {2, 3};

  template <typename T>
  static std::shared_ptr<DNNTensor> GetMemoryTensor(
    const std::vector<T>& embeddings,
    hbDNNTensorProperties &tensor_properties);

  int FeedFromLocal();

  std::string ai_msg_pub_topic_name_ = "/perception/segmentation/edgetam";
  rclcpp::Publisher<ai_msgs::msg::PerceptionTargets>::SharedPtr ai_msg_publisher_ =
      nullptr;

  std::string ai_msg_sub_topic_name_ = "/hobot_dnn_detection";
  rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr
      ai_msg_subscription_ = nullptr;
  void AiMsgProcess(const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg);

#ifdef SHARED_MEM_ENABLED
  void SharedMemImgProcess(
      const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr msg);
  rclcpp::Subscription<hbm_img_msgs::msg::HbmMsg1080P>::ConstSharedPtr
      sharedmem_img_subscription_ = nullptr;
  std::string sharedmem_img_topic_name_ = "/hbmem_img";
#endif
  void RosImgProcess(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  rclcpp::Subscription<sensor_msgs::msg::Image>::ConstSharedPtr
      ros_img_subscription_ = nullptr;
  std::string ros_img_topic_name_ = "/image_raw";
};

#endif  // MONO_EDGETAM_TRACK_NODE_H_
