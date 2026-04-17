[English](./README.md) | 简体中文

# 功能介绍

`mono_edgetam_track` 是一个基于 EdgeTAM 模型、通过 `dnn_node` 部署的 ROS2 跟踪分割示例。
该工程支持本地图片推理和订阅图片流推理，并通过 `ai_msgs::msg::PerceptionTargets` 发布分割结果。

# 开发环境

- 编程语言: C/C++
- 开发平台: S100
- 系统版本: Ubuntu 22.04
- 编译工具链: GCC 11.4.0

# 编译

该 package 支持在板端使用 `colcon` 编译，也可集成到平台交叉编译流程中。

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

- `-DPLATFORM_S100=ON`：编译 S100 平台版本。
- 未显式指定时默认按 S100 编译。

2、共享内存选项

- 工程默认打开共享内存图片订阅能力（`SHARED_MEM_ENABLED`）。
- 共享内存模式依赖 `hbm_img_msgs` 及相关运行时组件。

## Ubuntu系统上编译

1、编译环境确认

- 已 source ROS2 和 TogetherROS 环境。
- 已安装 `colcon`。
- 工作空间内相关依赖 package 可用。

2、编译

- 编译命令：`colcon build --packages-select mono_edgetam_track`

## docker交叉编译

1、编译环境确认

- 在docker中编译, 并且docker中已经安装好TogetherROS。docker安装、交叉编译说明、TogetherROS编译和部署说明详见机器人开发平台robot_dev_config repo中的README.md。
- 已编译dnn node package
- 已编译hbm_img_msgs package（编译方法见Dependency部分）

2、编译

- 编译命令：

  ```shell
  # RDK S100
  bash robot_dev_config/build.sh -p S100 -s mono_edgetam_track
  ```

- 编译选项中默认打开了shared mem通信方式。

# 使用介绍

## 依赖

- `mipi_cam` 或 `hobot_usb_cam` 或 `hobot_image_publisher`：图片输入源
- `hobot_codec`：图片编解码桥接
- `hobot_shm`：共享内存运行环境
- `websocket`：图片与 AI 结果可视化

## 参数

| 参数名 | 解释 | 是否必须 | 类型 | 默认值 |
| --- | --- | --- | --- | --- |
| `feed_type` | 图片来源模式，`0`：本地图片；`1`：订阅图片话题 | 否 | int | 0 |
| `is_sync_mode` | 推理模式，`0`：异步；`1`：同步 | 否 | int | 0 |
| `is_shared_mem_sub` | `feed_type=1` 时的订阅模式，`1`：共享内存；`0`：ROS图片话题 | 否 | int | 0 |
| `is_overwrite_features` | 推理结束后是否覆盖 features 在本地 | 否 | int | 0 |
| `dump_render_img` | 渲染图保存开关（当前代码路径预留） | 否 | int | 0 |
| `model_file_name` | EdgeTAM 模型文件路径 | 否 | string | `model_track_step_s7.hbm` |
| `init_mem_feat_path` | 初始 memory feature 文件路径 | 否 | string | `cond_maskmem_features.bin` |
| `init_mem_pos_path` | 初始 memory pos enc 文件路径 | 否 | string | `cond_maskmem_pos_enc.bin` |
| `init_mem_ptr_path` | 初始 memory object ptr 文件路径 | 否 | string | `cond_obj_ptr.bin` |
| `ros_img_topic_name` | 非共享内存订阅时的 ROS 图片话题名 | 否 | string | `/image_raw` |
| `ai_msg_pub_topic_name` | `feed_type=1` 时发布 `ai_msgs/PerceptionTargets`分割的话题 | 否 | string | `/perception/track/edgetam` |

## 运行

## Ubuntu系统上运行

方式1：可执行文件启动

```shell
export COLCON_CURRENT_PREFIX=./install
source /opt/ros/humble/setup.bash

source ./install/local_setup.bash

# 下载和拷贝运行所需配置
cp -r install/lib/mono_edgetam_track/config/* .
wget https://archive.d-robotics.cc/downloads/models/edgetam/s100/model_track_step_s7.hbm

# 运行模式1：本地图片推理
ros2 run mono_edgetam_track mono_edgetam_track --ros-args -p feed_type:=0

# 运行模式2：共享内存订阅图片推理
ros2 run mono_edgetam_track mono_edgetam_track --ros-args -p feed_type:=1 -p is_shared_mem_sub:=1
```

方式2：launch 启动（推荐）

```shell
export COLCON_CURRENT_PREFIX=./install
source /opt/ros/humble/setup.bash
source ./install/setup.bash

# 下载和拷贝运行所需配置
cp -r install/lib/mono_edgetam_track/config/* .
wget https://archive.d-robotics.cc/downloads/models/edgetam/s100/model_track_step_s7.hbm

# 选择相机来源: export CAM_TYPE=mipi/usb/fb
export CAM_TYPE=fb

# 启动完整链路：相机 + 编解码 + mono_edgetam_track + websocket
ros2 launch mono_edgetam_track mono_edgetam_track.launch.py edgetam_is_overwrite_features:=0
```

## Launch 参数

| 参数名 | 解释 | 默认值 |
| --- | --- | --- |
| `edgetam_image_width` | 相机/图片发布节点输入宽度 | 960 |
| `edgetam_image_height` | 相机/图片发布节点输入高度 | 540 |
| `edgetam_ai_msg_pub_topic_name` | 用于 websocket 展示的 AI 消息发布话题 | `/perception/track/edgetam` |
| `edgetam_is_overwrite_features` | 推理结束后是否覆盖 features 在本地 | 0 |
| `publish_image_source` | 回灌模式（`CAM_TYPE=fb`）使用的图片列表/路径 | `bedroom/images.list` |

# 结果分析

## RDK 结果展示

运行命令：
`ros2 run mono_edgetam_track mono_edgetam_track --ros-args -p feed_type:=0 -p dump_render_img:=1`

典型启动日志包括：

- 参数打印（`mono_edgetam_track params`）
- 模型名称输出
- 图像接收日志（`Recved img encoding ...`）
- 分割发布与性能统计（`Sub img fps`、`Smart fps`、pipeline 延迟）

```bash
[UCP]: log level = 3
[UCP]: UCP version = 3.13.6
[VP]: log level = 3
[DNN]: log level = 3
[HPL]: log level = 3
[UCPT]: log level = 6
[WARN] [1776416159.725947630] [mono_edgetam_track]: mono_edgetam_track params:
 feed_type(0:local, 1:sub): 0
 local_image_list: bedroom/images.list
 is_sync_mode: 0
 is_shared_mem_sub: 0
 dump_render_img: 1
 is_overwrite_features: 0
 model_file_name: model_track_step_s7.hbm
 init_mem_feat_path: cond_maskmem_features.bin
 init_mem_pos_path: cond_maskmem_pos_enc.bin
 init_mem_ptr_path: cond_obj_ptr.bin
 ros_img_topic_name: /image_raw
 ai_msg_pub_topic_name: /perception/track/edgetam
[INFO] [1776416159.726244735] [dnn]: Node init.
[INFO] [1776416159.726294836] [dnn]: Model init.
[BPU][[BPU_MONITOR]][281473721020448][INFO]BPULib verison(2, 2, 15)[f21ee84]!
[DNN]: 3.13.6_(4.7.5 HBRT)
[INFO] [1776416160.829687463] [dnn]: The model input 0 width is 1 and height is 1024
[INFO] [1776416160.829744939] [dnn]: The model input 1 width is 2 and height is 512
[INFO] [1776416160.829771865] [dnn]: The model input 2 width is 0 and height is 2
[INFO] [1776416160.829787415] [dnn]: The model input 3 width is 0 and height is 0
[INFO] [1776416160.829798615] [dnn]: The model input 4 width is 64 and height is 512
[INFO] [1776416160.829809565] [dnn]: The model input 5 width is 64 and height is 512
[INFO] [1776416160.829820415] [dnn]: The model input 6 width is 0 and height is 256
[INFO] [1776416160.829928042] [dnn]:
Model Info:
name: model_track_step_s7.
[input]
 - (0) Layout: NONE, Shape: [1, 1024, 1024, 1], Type: HB_DNN_TENSOR_TYPE_U8.
 - (1) Layout: NONE, Shape: [1, 512, 512, 2], Type: HB_DNN_TENSOR_TYPE_U8.
 - (2) Layout: NONE, Shape: [1, 1, 2, 0], Type: HB_DNN_TENSOR_TYPE_S32.
 - (3) Layout: NONE, Shape: [1, 1, 0, 0], Type: HB_DNN_TENSOR_TYPE_S32.
 - (4) Layout: NONE, Shape: [7, 1, 512, 64], Type: HB_DNN_TENSOR_TYPE_F32.
 - (5) Layout: NONE, Shape: [7, 1, 512, 64], Type: HB_DNN_TENSOR_TYPE_F32.
 - (6) Layout: NONE, Shape: [7, 1, 256, 0], Type: HB_DNN_TENSOR_TYPE_F32.
[output]
 - (0) Layout: NONE, Shape: [1, 1, 256, 256], Type: HB_DNN_TENSOR_TYPE_F32.
 - (1) Layout: NONE, Shape: [7, 1, 512, 64], Type: HB_DNN_TENSOR_TYPE_F32.
 - (2) Layout: NONE, Shape: [7, 1, 512, 64], Type: HB_DNN_TENSOR_TYPE_F32.
 - (3) Layout: NONE, Shape: [7, 1, 256, 0], Type: HB_DNN_TENSOR_TYPE_F32.

[INFO] [1776416160.830000768] [dnn]: Task init.
[INFO] [1776416160.831729197] [dnn]: Set task_num [4]
[WARN] [1776416160.831775523] [mono_edgetam_track]: Model name: model_track_step_s7
[WARN] [1776416160.835186604] [mono_edgetam_track]: Create ai msg publisher with topic_name: /perception/track/edgetam
[INFO] [1776416160.928698002] [mono_edgetam_track]: Update mem_feat, mem_pos, mem_ptr data from output tensors
[WARN] [1776416160.983582111] [mono_edgetam_track]: Save Render Image: render_0.jpg
[INFO] [1776416161.037302099] [mono_edgetam_track]: Update mem_feat, mem_pos, mem_ptr data from output tensors
[WARN] [1776416161.037352025] [mono_edgetam_track]: Save Render Image: render_1.jpg
[INFO] [1776416161.091765499] [mono_edgetam_track]: Update mem_feat, mem_pos, mem_ptr data from output tensors
[WARN] [1776416161.093659355] [mono_edgetam_track]: Save Render Image: render_2.jpg
```

## 渲染结果
![image](img/render_0.jpg)


![image](img/render_50.jpg)


![image](img/render_100.jpg)


![image](img/render_150.jpg)