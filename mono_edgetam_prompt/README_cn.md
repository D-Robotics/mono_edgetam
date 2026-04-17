[English](./README.md) | 简体中文

# 功能介绍

`mono_edgetam_prompt` 是 EdgeTAM 跟踪链路中的 **prompt 初始化** 阶段：基于输入图像与点提示（坐标、标签）做 prompt 模型推理，将 memory 相关张量 **落盘为 bin 文件** 供下游使用；在 `feed_type=1` 时还会 **订阅图像数据和上游检测 AI 消息** 更新提示点，并 **发布** 分割与目标 ROI 的 `ai_msgs` 结果。

简言之：**先用 `mono_edgetam_prompt` 生成 prompt / memory 初始化结果，再交给 `mono_edgetam_track` 做持续跟踪。**

# 开发环境

- 编程语言: C/C++
- 开发平台: S100
- 系统版本: Ubuntu 22.04
- 编译工具链: GCC 11.4.0

# 编译

## 依赖库

- OpenCV: 4.x

ROS package：

- rclcpp
- dnn_node
- ai_msgs
- sensor_msgs
- hbm_img_msgs
- hobot_cv

## 编译选项

1、平台选项

- `-DPLATFORM_S100=ON`：编译 RDK S100 平台版本。
- 未指定时默认按 S100 编译。

2、共享内存选项

- 工程默认打开共享内存订阅支持（`SHARED_MEM_ENABLED`）。
- 共享内存模式依赖 `hbm_img_msgs` 及对应运行时组件。

## Ubuntu 系统上编译

```shell
colcon build --packages-select mono_edgetam_prompt
```

## docker交叉编译

1、编译环境确认

- 在docker中编译, 并且docker中已经安装好TogetherROS。docker安装、交叉编译说明、TogetherROS编译和部署说明详见机器人开发平台robot_dev_config repo中的README.md。
- 已编译dnn node package
- 已编译hbm_img_msgs package（编译方法见Dependency部分）

2、编译

- 编译命令：

  ```shell
  # RDK S100
  bash robot_dev_config/build.sh -p S100 -s mono_edgetam_prompt
  ```

- 编译选项中默认打开了shared mem通信方式。

# 使用介绍

## 依赖

- `mipi_cam` / `hobot_usb_cam` / `hobot_image_publisher`：图像输入源
- `hobot_codec`：编解码桥接（launch 中在共享内存与 ROS 图像话题之间衔接）
- `hobot_shm`：共享内存运行环境
- `websocket`：图像与 AI 消息可视化

## 参数

参数名与 `mono_edgetam_prompt_node.cpp` 中 `declare_parameter` 一致。

| 参数名 | 解释 | 是否必须 | 类型 | 默认值 |
| --- | --- | --- | --- | --- |
| `feed_type` | 输入模式：`0` 本地图片，`1` 订阅图像（并配合 AI 订阅/发布） | 否 | int | `0` |
| `is_sync_mode` | DnnNode 推理：`0` 异步（线程池），`1` 同步（同线程 `RunImpl`） | 否 | int | `0` |
| `is_shared_mem_sub` | `feed_type=1` 时：`1` 订阅共享内存图像，`0` 订阅 `sensor_msgs/Image` | 否 | int | `0` |
| `prompt_mode` | 点提示预设：`0` 框类标签（`2`/`3`）及默认坐标；`1` 点类标签 `1` 及另一组坐标 | 否 | int | `0` |
| `dump_render_img` | `feed_type=0` 且开启时：在当前工作目录保存渲染图 `render.jpeg` | 否 | int | `0` |
| `model_file_name` | Prompt 模型（`.hbm`）路径 | 否 | string | `model_prompt_to_memory_points.hbm` |
| `dump_mem_feat_path` | 推理后将 memory feature 张量写入的文件路径 | 否 | string | `cond_maskmem_features.bin` |
| `dump_mem_ptr_path` | 推理后将 memory pointer 张量写入的文件路径 | 否 | string | `cond_obj_ptr.bin` |
| `image` | `feed_type=0` 时本地图片路径 | 否 | string | `bedroom/00000.jpg` |
| `ros_img_topic_name` | `feed_type=1` 且 `is_shared_mem_sub=0` 时的 ROS 图像话题 | 否 | string | `/image_raw` |
| `ai_msg_pub_topic_name` | `feed_type=1` 时发布 `ai_msgs/PerceptionTargets`（分割 + 目标）的话题 | 否 | string | `/perception/segmentation/edgetam` |
| `ai_msg_sub_topic_name` | `feed_type=1` 时订阅上游检测（`ai_msgs/PerceptionTargets`）以更新点提示 | 否 | string | `/hobot_dnn_detection` |
| `sharedmem_img_topic_name` | `is_shared_mem_sub=1` 时的共享内存图像话题（需 `SHARED_MEM_ENABLED`） | 否 | string | `/hbmem_img` |

## 运行

### 直接运行可执行文件

```shell
source /opt/ros/humble/setup.bash
source ./install/setup.bash

# 根据实际安装路径进行拷贝
wget https://archive.d-robotics.cc/downloads/models/edgetam/s100/model_prompt_to_memory_points.hbm
wget https://archive.d-robotics.cc/downloads/models/edgetam/bedroom.tar
tar -xvf bedroom.tar

# 运行模式1：
# 使用本地jpg格式图片进行回灌预测
ros2 run mono_edgetam_prompt mono_edgetam_prompt --ros-args -p feed_type:=0 -p image:=bedroom/00000.jpg -p image_type:=0 -p dump_render_img:=1

# 运行模式2：使用shared mem通信方式(topic为/hbmem_img)进行预测。同时支持在另一个窗口发送ai msg话题(topic为/hobot_dnn_detection) 变更提示框
ros2 run mono_edgetam_prompt mono_edgetam_prompt --ros-args -p feed_type:=1 --ros-args --log-level warn -p prompt_mode:=0 -p ai_msg_sub_topic_name:="/hobot_dnn_detection"

ros2 topic pub /hobot_dnn_detection ai_msgs/msg/PerceptionTargets '{"targets": [{"rois": [{"rect": {"x_offset": 240, "y_offset": 135, "width": 480, "height": 270}, "type": "anything"}]}] }'

# 运行模式3：使用shared mem通信方式(topic为/hbmem_img)进行预测。同时支持在另一个窗口发送ai msg话题(topic为/hobot_dnn_detection) 变更提示点, 设置框的大小为0
ros2 run mono_edgetam_prompt mono_edgetam_prompt --ros-args -p feed_type:=1 --ros-args --log-level warn -p prompt_mode:=0 -p ai_msg_sub_topic_name:="/hobot_dnn_detection"

ros2 topic pub /hobot_dnn_detection ai_msgs/msg/PerceptionTargets '{"targets": [{"rois": [{"rect": {"x_offset": 210, "y_offset": 350, "width": 0, "height": 0}, "type": "anything"}, {"rect": {"x_offset": 250, "y_offset": 220, "width": 0, "height": 0}, "type": "anything"}]}] }'
```

### 使用 launch 启动（推荐）

```shell
source /opt/ros/humble/setup.bash
source ./install/setup.bash

# 根据实际安装路径进行拷贝
wget https://archive.d-robotics.cc/downloads/models/edgetam/s100/model_prompt_to_memory_points.hbm
wget https://archive.d-robotics.cc/downloads/models/edgetam/bedroom.tar
tar -xvf bedroom.tar

# export CAM_TYPE=mipi/usb/fb
export CAM_TYPE=fb
ros2 launch mono_edgetam_prompt mono_edgetam_prompt.launch.py edgetam_prompt_mode:=0
```

`mono_edgetam_prompt.launch.py` 中声明一致。

| 参数名 | 解释 | 默认值 |
| --- | --- | --- |
| `edgetam_image_width` | 相机 / 回灌发布节点图像宽度 | `960` |
| `edgetam_image_height` | 相机 / 回灌发布节点图像高度 | `540` |
| `edgetam_msg_pub_topic_name` | 传给节点的 `ai_msg_pub_topic_name` 及 websocket 智能结果话题 | `/perception/segmentation/edgetam` |
| `edgetam_prompt_mode` | 映射为节点参数 `prompt_mode` | `0` |
| `device` | 相机设备（USB：`/dev/video0`，MIPI：`F37` 等） | launch 内按平台默认 |
| `publish_image_source` | `CAM_TYPE=fb` 时的图片路径 | `bedroom/00000.jpg` |

**流水线概要：** MIPI / 回灌：相机 → 共享内存 → `hobot_codec` 编码（如 `/hbmem_img` → `/image`）→ 节点；USB：走 launch 中另一路 NV12 编解码分支。

# 结果分析

## RDK 结果展示

log：

运行命令：`ros2 run mono_edgetam_prompt mono_edgetam_prompt --ros-args -p feed_type:=0 -p dump_render_img:=1`

```shell
[UCP]: log level = 3
[UCP]: UCP version = 3.13.6
[VP]: log level = 3
[DNN]: log level = 3
[HPL]: log level = 3
[UCPT]: log level = 6
[WARN] [1776415725.063074658] [mono_edgetam_prompt]: mono_edgetam_prompt params:
 feed_type(0:local, 1:sub): 0
 image: bedroom/00000.jpg
 is_sync_mode: 0
 is_shared_mem_sub: 0
 prompt_mode: 0
 ros_img_topic_name: /image_raw
 ai_msg_pub_topic_name: /perception/segmentation/edgetam
 ai_msg_sub_topic_name: /hobot_dnn_detection
 model_file_name: model_prompt_to_memory_points.hbm
 dump_mem_feat_path: cond_maskmem_features.bin
 dump_mem_ptr_path: cond_obj_ptr.bin
 dump_render_img: 1
[INFO] [1776415725.063383821] [dnn]: Node init.
[INFO] [1776415725.063433173] [dnn]: Model init.
[BPU][[BPU_MONITOR]][281473663143968][INFO]BPULib verison(2, 2, 15)[f21ee84]!
[DNN]: 3.13.6_(4.7.5 HBRT)
[INFO] [1776415726.648634111] [dnn]: The model input 0 width is 1 and height is 1024
[INFO] [1776415726.648696689] [dnn]: The model input 1 width is 2 and height is 512
[INFO] [1776415726.648720040] [dnn]: The model input 2 width is 0 and height is 2
[INFO] [1776415726.648733791] [dnn]: The model input 3 width is 0 and height is 0
[INFO] [1776415726.648820119] [dnn]:
Model Info:
name: model_prompt_to_memory_points.
[input]
 - (0) Layout: NONE, Shape: [1, 1024, 1024, 1], Type: HB_DNN_TENSOR_TYPE_U8.
 - (1) Layout: NONE, Shape: [1, 512, 512, 2], Type: HB_DNN_TENSOR_TYPE_U8.
 - (2) Layout: NONE, Shape: [1, 2, 2, 0], Type: HB_DNN_TENSOR_TYPE_S32.
 - (3) Layout: NONE, Shape: [1, 2, 0, 0], Type: HB_DNN_TENSOR_TYPE_S32.
[output]
 - (0) Layout: NONE, Shape: [1, 1, 256, 256], Type: HB_DNN_TENSOR_TYPE_F32.
 - (1) Layout: NONE, Shape: [1, 512, 64, 0], Type: HB_DNN_TENSOR_TYPE_F32.
 - (2) Layout: NONE, Shape: [1, 256, 0, 0], Type: HB_DNN_TENSOR_TYPE_F32.

[INFO] [1776415726.648874196] [dnn]: Task init.
[INFO] [1776415726.650730222] [dnn]: Set task_num [4]
[WARN] [1776415726.650790725] [mono_edgetam_prompt]: Model name: model_prompt_to_memory_points
[WARN] [1776415726.788639263] [mono_edgetam_prompt]: Save Render Image: render.jpeg
```

## 渲染结果
![image](img/render.jpeg)