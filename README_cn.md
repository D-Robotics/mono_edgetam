# mono_edgetam

`mono_edgetam` 是基于[EdgeTAM](https://github.com/facebookresearch/EdgeTAM)在RDK平台上部署的应用工程。包含 EdgeTAM 跟踪链路中的两个子工程：

- `mono_edgetam_prompt`：负责 prompt 初始化，基于输入图像和点提示做模型推理，生成/发布初始化结果。
- `mono_edgetam_track`：负责连续跟踪分割，消费初始化信息后进行后续帧跟踪并发布结果。

## 平台支持情况

- [RDK S100](https://developer.d-robotics.cc/rdks100)

- [RDK S100P](https://developer.d-robotics.cc/rdks100)

## 子工程说明与链接

### 1) mono_edgetam_prompt

- 目录：`mono_edgetam_prompt/`
- 文档：
  - 中文：[`mono_edgetam_prompt/README_cn.md`](./mono_edgetam_prompt/README_cn.md)
  - 英文：[`mono_edgetam_prompt/README.md`](./mono_edgetam_prompt/README.md)
- 功能定位：EdgeTAM 链路的 prompt 初始化阶段，支持本地图和订阅图像模式。

### 2) mono_edgetam_track

- 目录：`mono_edgetam_track/`
- 文档：
  - 中文：[`mono_edgetam_track/README_cn.md`](./mono_edgetam_track/README_cn.md)
  - 英文：[`mono_edgetam_track/README.md`](./mono_edgetam_track/README.md)
- 功能定位：EdgeTAM 连续跟踪分割阶段，支持本地图和订阅图像流推理。

## 两个子工程配合使用（Launch 方式）

下面给出推荐的 launch 协同运行流程：先启动 `mono_edgetam_prompt` 通过检测框/检测点获取图像目标, 得到特征信息。再启动 `mono_edgetam_track`, 加载 `mono_edgetam_prompt` 保存的特征, 开始跟踪任务。两个节点不支持同时启动。

### 0) 准备示例所用数据

```shell
wget https://archive.d-robotics.cc/downloads/models/edgetam/bedroom.tar
tar -xvf bedroom.tar
```

### 1) 启动 mono_edgetam_prompt

```bash
source /opt/ros/humble/setup.bash
source ./install/setup.bash

# 下载模型
wget https://archive.d-robotics.cc/downloads/models/edgetam/s100/model_prompt_to_memory_points.hbm

# 相机来源可选: mipi / usb / fb
export CAM_TYPE=fb
ros2 launch mono_edgetam_prompt mono_edgetam_prompt.launch.py edgetam_prompt_mode:=0
```

示例中推理的结果会渲染到Web上, 在PC端的浏览器输入http://IP:8000 即可查看图像和算法渲染效果（IP为RDK的IP地址）, 打开界面右上角设置, 选中”Full Image Segmentation“选项, 可以显示渲染效果。

![image](render_frame0.png)

### 2) 启动 mono_edgetam_track

```bash
source /opt/ros/humble/setup.bash
source ./install/setup.bash

# 下载模型
wget https://archive.d-robotics.cc/downloads/models/edgetam/s100/model_track_step_s7.hbm

# 建议与 prompt 使用一致的图像来源
export CAM_TYPE=fb
ros2 launch mono_edgetam_track mono_edgetam_track.launch.py edgetam_is_overwrite_features:=0
```

示例中推理的结果会渲染到Web上, 在PC端的浏览器输入http://IP:8000 即可查看图像和算法渲染效果（IP为RDK的IP地址）, 打开界面右上角设置, 选中”Full Image Segmentation“选项, 可以显示渲染效果。

![image](render_frames.gif)