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

#include <opencv2/opencv.hpp>

#include "include/prompt_output_parser.h"

static void ResizeLogitsBilinearAlignCornersFalse(const float *src, int in_h, int in_w, int out_h,
                                                   int out_w, float *dst) {
  const float scale_y = static_cast<float>(in_h) / static_cast<float>(out_h);
  const float scale_x = static_cast<float>(in_w) / static_cast<float>(out_w);
  for (int oy = 0; oy < out_h; ++oy) {
    const float in_y = (static_cast<float>(oy) + 0.5f) * scale_y - 0.5f;
    int y0 = static_cast<int>(std::floor(in_y));
    int y1 = y0 + 1;
    y0 = std::max(0, std::min(in_h - 1, y0));
    y1 = std::max(0, std::min(in_h - 1, y1));
    const float wy = in_y - static_cast<float>(y0);
    for (int ox = 0; ox < out_w; ++ox) {
      const float in_x = (static_cast<float>(ox) + 0.5f) * scale_x - 0.5f;
      int x0 = static_cast<int>(std::floor(in_x));
      int x1 = x0 + 1;
      x0 = std::max(0, std::min(in_w - 1, x0));
      x1 = std::max(0, std::min(in_w - 1, x1));
      const float wx = in_x - static_cast<float>(x0);

      const float x00 = src[y0 * in_w + x0];
      const float x01 = src[y0 * in_w + x1];
      const float x10 = src[y1 * in_w + x0];
      const float x11 = src[y1 * in_w + x1];

      const float top = (1.0f - wx) * x00 + wx * x01;
      const float bottom = (1.0f - wx) * x10 + wx * x11;
      dst[oy * out_w + ox] = (1.0f - wy) * top + wy * bottom;
    }
  }
}

int PromptOutputParser::RenderSeg(const std::string &img_path,
                     const Parsing &seg, 
                     const std::string &saving_path) {
  static uint8_t bgr_putpalette[] = {
      0, 0, 0, 0, 127, 245};

  if (seg.valid_w <= 0 || seg.valid_h <= 0 ||
      static_cast<int>(seg.seg.size()) != seg.valid_w * seg.valid_h) {
    return -2;
  }

  cv::Mat bgr_img;
  bgr_img = cv::imread(img_path);

  const double alpha_f = 0.5;
  for (int h = 0; h < seg.valid_h; ++h) {
    for (int w = 0; w < seg.valid_w; ++w) {
      const uint8_t id = seg.seg[h * seg.valid_w + w];

      // 只在前景类别叠加（假设 id=0 是背景）
      if (id == 0) continue;

      cv::Vec3b &pixel = bgr_img.at<cv::Vec3b>(h, w);

      uint8_t b = bgr_putpalette[id * 3];
      uint8_t g = bgr_putpalette[id * 3 + 1];
      uint8_t r = bgr_putpalette[id * 3 + 2];

      // alpha blending（只改当前像素）
      pixel[0] = static_cast<uint8_t>(pixel[0] * alpha_f + b * (1 - alpha_f));
      pixel[1] = static_cast<uint8_t>(pixel[1] * alpha_f + g * (1 - alpha_f));
      pixel[2] = static_cast<uint8_t>(pixel[2] * alpha_f + r * (1 - alpha_f));
    }
  }

  if (!cv::imwrite(saving_path, bgr_img)) {
    return -1;
  }
  return 0;
}

int32_t PromptOutputParser::Parse(
    std::shared_ptr<DnnParserResult> &result,
    std::shared_ptr<DNNTensor> &output_tensor,
    const int img_h,
    const int img_w) {
    
  if (!result) {
    result = std::make_shared<DnnParserResult>();
  }

  auto *low_res =
    reinterpret_cast<float *>(output_tensor->sysMem.virAddr);

  std::vector<float> low_res_up_f32(img_h * img_w);
  ResizeLogitsBilinearAlignCornersFalse(low_res, low_res_h_, low_res_w_, img_h,
                                        img_w, low_res_up_f32.data());

  std::vector<int8_t> low_res_mask(img_h * img_w);
  std::vector<float> low_res_mask_float(img_h * img_w);
  for (size_t i = 0; i < low_res_mask.size(); ++i) {
    low_res_mask[i] = (low_res_up_f32[i] > 0.0f) ? 1 : 0;
    low_res_mask_float[i] = (low_res_up_f32[i] > 0.0f) ? 1.0 : 0.0;
  }

  Parsing seg;
  seg.width = img_w;
  seg.height = img_h;
  seg.valid_w = img_w;
  seg.valid_h = img_h;
  seg.channel = 1;
  seg.num_classes = 1;
  seg.seg = low_res_mask;
  seg.data = low_res_mask_float;

  Perception perception;
  perception.type = Perception::SEG;
  perception.seg = seg;
  result->perception = perception;

  return 0;
}
