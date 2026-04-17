English | [简体中文](./README_cn.md)

# Feature Introduction

`mono_edgetam_track` is a ROS2 tracking-and-segmentation example based on the EdgeTAM model and deployed through `dnn_node`.
It supports both local-image inference and subscribed image-stream inference, and publishes segmentation results via `ai_msgs::msg::PerceptionTargets`.

# Development Environment

- Programming language: C/C++
- Development platform: S100
- System version: Ubuntu 22.04
- Compiler toolchain: GCC 11.4.0

# Build

This package can be built on device with `colcon`, and can also be integrated into your platform cross-compilation workflow.

## Dependencies

- OpenCV: 4.x

ROS packages:

- rclcpp
- dnn_node
- ai_msgs
- sensor_msgs
- hbm_img_msgs
- hobot_cv

## Build Options

1. Platform options

- `-DPLATFORM_S100=ON`: build for S100.
- If not explicitly specified, S100 is used by default.

2. Shared-memory option

- Shared-memory image subscription is enabled by default (`SHARED_MEM_ENABLED`).
- Shared-memory mode depends on `hbm_img_msgs` and related runtime components.

## Build on Ubuntu

1. Verify the build environment

- ROS2 and TogetherROS environments are sourced.
- `colcon` is installed.
- Required dependency packages are available in the workspace.

2. Build command

- `colcon build --packages-select mono_edgetam_track`

## Docker Cross-Compilation

1. Compilation Environment Verification

- Compilation within docker, and TogetherROS has been installed in the docker environment. For instructions on docker installation, cross-compilation, TogetherROS compilation, and deployment, please refer to the README.md in the robot development platform's robot_dev_config repo.
- The dnn node package has been compiled.
- The hbm_img_msgs package has been compiled (see Dependency section for compilation methods).

2. Compilation

- Compilation command:

  ```shell
  # RDK S100
  bash robot_dev_config/build.sh -p S100 -s mono_edgetam_track
  ```

- Shared memory communication method is enabled by default in the compilation options.

# Usage

## Dependencies

- `mipi_cam` / `hobot_usb_cam` / `hobot_image_publisher`: image input source
- `hobot_codec`: image encode/decode bridge
- `hobot_shm`: shared-memory runtime environment
- `websocket`: image and AI-result visualization

## Parameters

| Parameter | Description | Required | Type | Default |
| --- | --- | --- | --- | --- |
| `feed_type` | Image source mode, `0`: local image; `1`: subscribed image topic | No | int | 0 |
| `is_sync_mode` | Inference mode, `0`: async; `1`: sync | No | int | 0 |
| `is_shared_mem_sub` | Subscription mode when `feed_type=1`, `1`: shared memory; `0`: ROS image topic | No | int | 0 |
| `is_overwrite_features` | Whether to overwrite local feature files after inference | No | int | 0 |
| `dump_render_img` | Render-image dump switch (currently reserved in the code path) | No | int | 0 |
| `model_file_name` | EdgeTAM model file path | No | string | `model_track_step_s7.hbm` |
| `init_mem_feat_path` | Initial memory feature file path | No | string | `cond_maskmem_features.bin` |
| `init_mem_pos_path` | Initial memory position-encoding file path | No | string | `cond_maskmem_pos_enc.bin` |
| `init_mem_ptr_path` | Initial memory object-pointer file path | No | string | `cond_obj_ptr.bin` |
| `ros_img_topic_name` | ROS image topic name when not using shared-memory subscription | No | string | `/image_raw` |
| `ai_msg_pub_topic_name` | Topic for publishing segmentation `ai_msgs/PerceptionTargets` when `feed_type=1` | No | string | `/perception/track/edgetam` |

## Run

## Run on Ubuntu

Method 1: Start executable directly

```shell
export COLCON_CURRENT_PREFIX=./install
source /opt/ros/humble/setup.bash

source ./install/local_setup.bash

# Download and copy required runtime configs
cp -r install/lib/mono_edgetam_track/config/* .
wget https://archive.d-robotics.cc/downloads/models/edgetam/s100/model_track_step_s7.hbm

# Mode 1: local image inference
ros2 run mono_edgetam_track mono_edgetam_track --ros-args -p feed_type:=0

# Mode 2: shared-memory subscribed image inference
ros2 run mono_edgetam_track mono_edgetam_track --ros-args -p feed_type:=1 -p is_shared_mem_sub:=1
```

Method 2: Start with launch (recommended)

```shell
export COLCON_CURRENT_PREFIX=./install
source /opt/ros/humble/setup.bash
source ./install/setup.bash

# Download and copy required runtime configs
cp -r install/lib/mono_edgetam_track/config/* .
wget https://archive.d-robotics.cc/downloads/models/edgetam/s100/model_track_step_s7.hbm

# Select camera source: export CAM_TYPE=mipi/usb/fb
export CAM_TYPE=fb

# Start full pipeline: camera + codec + mono_edgetam_track + websocket
ros2 launch mono_edgetam_track mono_edgetam_track.launch.py edgetam_is_overwrite_features:=0
```

## Launch Parameters

| Parameter | Description | Default |
| --- | --- | --- |
| `edgetam_image_width` | Input width of camera/image publisher node | 960 |
| `edgetam_image_height` | Input height of camera/image publisher node | 540 |
| `edgetam_ai_msg_pub_topic_name` | AI message publish topic used by websocket visualization | `/perception/track/edgetam` |
| `edgetam_is_overwrite_features` | Whether to overwrite local feature files after inference | 0 |
| `publish_image_source` | Image list/path for replay mode (`CAM_TYPE=fb`) | `bedroom/images.list` |

# Result Analysis

## RDK Result Display

Run command:
`ros2 run mono_edgetam_track mono_edgetam_track --ros-args -p feed_type:=0 -p dump_render_img:=1`

Typical startup logs include:

- Parameter dump (`mono_edgetam_track params`)
- Model name output
- Image receive logs (`Recved img encoding ...`)
- Segmentation publish and performance stats (`Sub img fps`, `Smart fps`, pipeline latency)

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

## Render Result
![image](img/render_0.jpg)


![image](img/render_50.jpg)


![image](img/render_100.jpg)


![image](img/render_150.jpg)