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

#ifndef IMAGE_PROC_H
#define IMAGE_PROC_H

#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

#include "dnn_node/dnn_node_data.h"
#include "dnn_node/util/image_proc.h"

#define ALIGNED_2E(w, alignment) \
  ((static_cast<uint32_t>(w) + (alignment - 1U)) & (~(alignment - 1U)))
#define ALIGN_16(w) ALIGNED_2E(w, 16U)
#define ALIGN_32(w) ALIGNED_2E(w, 32U)
#define ALIGN_64(w) ALIGNED_2E(w, 64U)

#ifdef PLATFORM_S100
#define BPU_ALIGN(value) ALIGN_32(value)
#elif defined(PLATFORM_S600)
#define BPU_ALIGN(value) ALIGN_64(value)
#else
#define BPU_ALIGN(value) ALIGN_32(value)
#endif

using hobot::dnn_node::DNNTensor;

template <typename T>
static bool WriteBinaryFile(const std::string &path,
                            const T* data,
                            size_t count) {
  if (!data || count == 0) return false;

  std::ofstream ofs(path, std::ios::binary);
  if (!ofs) return false;

  const char *ptr = reinterpret_cast<const char *>(data);
  std::streamsize size_bytes =
      static_cast<std::streamsize>(count * sizeof(T));

  ofs.write(ptr, size_bytes);

  return ofs.good();
}

class ImageProc {
 public:
  static std::pair<std::shared_ptr<DNNTensor>, std::shared_ptr<DNNTensor>> 
    GetNV12TensorsFromNV12Img(
      const char* in_img_data,
      hbDNNTensorProperties &tensor_y_properties,
      hbDNNTensorProperties &tensor_uv_properties,
      const int& scaled_img_height,
      const int& scaled_img_width);

  static std::pair<std::shared_ptr<DNNTensor>, std::shared_ptr<DNNTensor>>
    GetNV12TensorFromNV12(const std::string &image_file,
                          hbDNNTensorProperties &tensor_y_properties,
                          hbDNNTensorProperties &tensor_uv_properties,
                          const int scaled_img_height,
                          const int scaled_img_width);
};

#endif  // IMAGE_PROC_H