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

#include <algorithm>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "include/image_proc.h"

std::pair<std::shared_ptr<DNNTensor>, std::shared_ptr<DNNTensor>> 
  ImageProc::GetNV12TensorsFromNV12Img(
    const char *in_img_data,
    hbDNNTensorProperties &tensor_y_properties,
    hbDNNTensorProperties &tensor_uv_properties,
    const int &scaled_img_height,
    const int &scaled_img_width) {
  auto *y = new hbUCPSysMem;
  auto *uv = new hbUCPSysMem;
  auto w_stride = BPU_ALIGN(scaled_img_width);
  hbUCPMallocCached(y, scaled_img_height * w_stride, 0);
  hbUCPMallocCached(uv, scaled_img_height / 2 * w_stride, 0);
  //内存初始化
  memset(y->virAddr, 0, scaled_img_height * w_stride);
  memset(uv->virAddr, 0, scaled_img_height / 2 * w_stride);
  // 根据图像和模型输入的最短的长和宽去图像里截取部分
  const uint8_t *data = reinterpret_cast<const uint8_t *>(in_img_data);
  auto *hb_y_addr = reinterpret_cast<uint8_t *>(y->virAddr);
  auto *hb_uv_addr = reinterpret_cast<uint8_t *>(uv->virAddr);

  memcpy(hb_y_addr, data, scaled_img_height * scaled_img_width);
  memcpy(hb_uv_addr, data + scaled_img_height * scaled_img_width, scaled_img_height / 2 * scaled_img_width);

  hbUCPMemFlush(y, HB_SYS_MEM_CACHE_CLEAN);
  hbUCPMemFlush(uv, HB_SYS_MEM_CACHE_CLEAN);

  auto y_tensor = new DNNTensor;
  y_tensor->sysMem.virAddr = reinterpret_cast<void *>(y->virAddr);
  y_tensor->sysMem.phyAddr = y->phyAddr;
  y_tensor->sysMem.memSize = scaled_img_height * scaled_img_width;
  y_tensor->properties = tensor_y_properties;
  y_tensor->properties.alignedByteSize = scaled_img_height * scaled_img_width;
  y_tensor->properties.stride[0] = scaled_img_height * scaled_img_width;
  y_tensor->properties.stride[1] = scaled_img_width;

  auto uv_tensor = new DNNTensor;
  uv_tensor->sysMem.virAddr = reinterpret_cast<void *>(uv->virAddr);
  uv_tensor->sysMem.phyAddr = uv->phyAddr;
  uv_tensor->sysMem.memSize = scaled_img_height / 2 * scaled_img_width;
  uv_tensor->properties = tensor_uv_properties;
  uv_tensor->properties.alignedByteSize = scaled_img_height / 2 * scaled_img_width;
  uv_tensor->properties.stride[0] = scaled_img_height / 2 * scaled_img_width;
  uv_tensor->properties.stride[1] = scaled_img_width;

  return std::make_pair(
    std::shared_ptr<DNNTensor>(y_tensor, [y](DNNTensor *tensor) {
      hbUCPFree(y);
      delete y;
      delete tensor;
    }),
    std::shared_ptr<DNNTensor>(uv_tensor, [uv](DNNTensor *tensor) {
      hbUCPFree(uv);
      delete uv;
      delete tensor;
    })
  );
}

std::pair<std::shared_ptr<DNNTensor>, std::shared_ptr<DNNTensor>>
 ImageProc::GetNV12TensorFromNV12(const std::string &image_file,
                                  hbDNNTensorProperties &tensor_y_properties,
                                  hbDNNTensorProperties &tensor_uv_properties,
                                  const int scaled_img_height,
                                  const int scaled_img_width) {
  cv::Mat nv12_mat;
  cv::Mat bgr_mat = cv::imread(image_file, cv::IMREAD_COLOR);

  cv::Mat resized;
  cv::resize(bgr_mat, resized, cv::Size(scaled_img_width, scaled_img_height), 0, 0, cv::INTER_LINEAR);
  
  auto ret = hobot::dnn_node::ImageProc::BGRToNv12(resized, nv12_mat);
  int original_img_height = bgr_mat.rows;
  int original_img_width = bgr_mat.cols;

  auto *y = new hbUCPSysMem;
  auto *uv = new hbUCPSysMem;

  auto w_stride = BPU_ALIGN(scaled_img_width);
  hbUCPMallocCached(y, scaled_img_height * w_stride, 0);
  hbUCPMallocCached(uv, scaled_img_height / 2 * w_stride, 0);

  uint8_t *data = nv12_mat.data;
  auto *hb_y_addr = reinterpret_cast<uint8_t *>(y->virAddr);
  auto *hb_uv_addr = reinterpret_cast<uint8_t *>(uv->virAddr);

  memcpy(hb_y_addr, data, scaled_img_height * scaled_img_width);
  memcpy(hb_uv_addr, data + scaled_img_height * scaled_img_width, scaled_img_height / 2 * scaled_img_width);

  hbUCPMemFlush(y, HB_SYS_MEM_CACHE_CLEAN);
  hbUCPMemFlush(uv, HB_SYS_MEM_CACHE_CLEAN);

  auto y_tensor = new DNNTensor;
  y_tensor->sysMem.virAddr = reinterpret_cast<void *>(y->virAddr);
  y_tensor->sysMem.phyAddr = y->phyAddr;
  y_tensor->sysMem.memSize = scaled_img_height * scaled_img_width;
  y_tensor->properties = tensor_y_properties;
  // y_tensor->properties.tensorType = HB_DNN_TENSOR_TYPE_U8;
  y_tensor->properties.alignedByteSize = scaled_img_height * scaled_img_width;
  y_tensor->properties.stride[0] = scaled_img_height * scaled_img_width;
  y_tensor->properties.stride[1] = scaled_img_width;

  auto uv_tensor = new DNNTensor;
  uv_tensor->sysMem.virAddr = reinterpret_cast<void *>(uv->virAddr);
  uv_tensor->sysMem.phyAddr = uv->phyAddr;
  uv_tensor->sysMem.memSize = scaled_img_height / 2 * scaled_img_width;
  uv_tensor->properties = tensor_uv_properties;
  uv_tensor->properties.alignedByteSize = scaled_img_height / 2 * scaled_img_width;
  uv_tensor->properties.stride[0] = scaled_img_height / 2 * scaled_img_width;
  uv_tensor->properties.stride[1] = scaled_img_width;

  return std::make_pair(
    std::shared_ptr<DNNTensor>(y_tensor, [y](DNNTensor *tensor) {
      hbUCPFree(y);
      delete y;
      delete tensor;
    }),
    std::shared_ptr<DNNTensor>(uv_tensor, [uv](DNNTensor *tensor) {
      hbUCPFree(uv);
      delete uv;
      delete tensor;
    })
  );
}

int32_t ImageProc::ReadImageList(const std::string& file_path,
                   std::vector<std::string>& images) {
    std::ifstream ifs(file_path);
    if (!ifs.is_open()) {
        return -1;
    }

    std::string line;
    while (std::getline(ifs, line)) {
        if (line.empty()) continue;  // 跳过空行
        images.push_back(line);
    }

    return 0;
}