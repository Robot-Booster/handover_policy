# handover_baseline

## 项目目的

本项目用于构建一套面向抓取/交接场景的 ROS2 感知与运动状态估计基线流程：  
通过双目深度、手部分割、点云筛选与目标状态估计，输出可供机械臂执行的抓取位姿与相关调试信息。

## 主体 Pipeline

主数据流如下（按运行链路）：

1. 相机驱动发布图像与相机参数（例如 RealSense）。
2. `cam_pre`（可选）对原始图像去畸变，发布 rect 图像与 rect `CameraInfo`。
3. `fastfoundation` 基于双目图像推理深度。
4. `hand_detector` 生成手/目标区域分割掩码。
5. `pc_processor` 融合深度 + 分割 + ROI/工作空间约束，输出目标点云与质心。
6. `handover_task` 结合目标点与 TCP 位姿，输出抓取位姿并编排交接任务。
7. `ur_robotiq` 与 `ur_robotiq_interfaces` 负责机械臂执行侧接口（位姿运动、夹爪控制）。

## 演示视频

- RViz 视角演示：`docs/video/rivz_light.gif`
- 跟踪流程演示：`docs/video/track_demo_light.gif`

![RViz 演示](docs/video/rivz_light.gif)

![跟踪流程演示](docs/video/track_demo_light.gif)

## 子功能包

| 包名 | 说明 |
|------|------|
| `cam_pre` | 相机图像去畸变预处理（C++）。 |
| `fastfoundation` | 双目深度推理节点。 |
| `hand_detector` | YOLO 分割节点。 |
| `pc_processor` | 点云处理与目标识别节点。 |
| `handover_task` | 抓取位姿估计与交接任务编排。 |
| `handover_task_interfaces` | 任务编排服务接口（`BasePolicy.srv`）。 |
| `ur_robotiq` | 机械臂控制侧 ROS2 节点。 |
| `ur_robotiq_interfaces` | 机械臂控制服务接口定义。 |

## 部署说明

> ⚠️ 部署前请先逐个阅读「各子包详细说明」（见下方链接），确认参数、话题、坐标系和资源路径都已对齐，再开始部署；否则很容易出现无输出、坐标错位、控制拒收等问题。

### 1) 环境与依赖

- Python 依赖请使用仓库根目录 `requmnet.txt` 安装。
- `numpy` 已锁定为 `1.26.*`。
- `torch` 需按本机环境（CPU/CUDA、驱动、CUDA 版本）单独安装，不在统一依赖中强制固定。

### 2) 编译

在工作空间根目录执行（按你的 ROS2 习惯）：

```bash
colcon build
source install/setup.bash
```

### 3) Fast-FoundationStereo 第三方部署说明（重要）

- `fastfoundation` 依赖第三方库 Fast-FoundationStereo，请按其官方说明自行完成部署。
- 模型权重与相关配置请按第三方说明自行下载，并放置到你配置的 `weight_dir`。
- 推荐根据你的硬件环境（GPU 型号、驱动、CUDA、TensorRT 版本）做 TensorRT 优化与 engine 构建，以获得更稳定的实时性能。

### 4) 启动顺序（建议）


```bash
# 1. 相机驱动
ros2 launch realsense2_camera rs_launch.py \
  config_file:=/home/ender/handover_baseline/scripts/d455_config.yaml \
  camera_name:=d455 camera_namespace:=left

# 2. 图像预处理（去畸变）
ros2 launch cam_pre cam_preprocess.launch.py \
  config_file:=/home/ender/handover_baseline/src/cam_pre/config/config.yaml \
  camera_name:=d455 namespace:=left

# 3. 机械臂状态/控制节点
ros2 launch ur_robotiq pose_tracker.launch.py \
  params_file:=/home/ender/handover_baseline/src/ur_robotiq/config/robot_config.yaml

# 4. 双目深度
ros2 launch fastfoundation launch_fastfoundation.launch.py \
  params_file:=/home/ender/handover_baseline/src/fastfoundation/config/d455.yaml \
  namespace:=d455

# 5. 分割
ros2 launch hand_detector hand_detector.launch.py \
  namespace:=d455 \
  config_file:=/home/ender/handover_baseline/src/hand_detector/config/config.yaml \
  image_topic:=/camera/d455/infra1/image_rect_raw

# 6. 点云处理/目标识别
ros2 launch pc_processor perception.launch.py \
  namespace:=d455 \
  config_file:=/home/ender/handover_baseline/src/pc_processor/config/d455_perception.yaml

# 7. 抓取位姿估计与任务编排
ros2 launch handover_task base_policy.launch.py \
  config_file:=/home/ender/handover_baseline/src/handover_task/config/base_policy.yaml

# 8. 触发一次交接任务
ros2 service call /base_policy/base_policy handover_task_interfaces/srv/BasePolicy "{trigger: true}"
```

### 5) 机械臂适配注意事项（重要）

- 本仓库默认控制接口以 `ur_robotiq` / `ur_robotiq_interfaces` 为参考实现。
- 你的机械臂若不是 UR+Robotiq，需单独实现自己的控制包。
- 新控制包建议对齐 `ur_robotiq_interfaces` 的服务语义与消息约定，保证上层流程最小改动可复用。

## 各子包详细文档

- [`cam_pre` 详细说明](docs/packages/cam_pre.md)
- [`fastfoundation` 详细说明](docs/packages/fastfoundation.md)
- [`hand_detector` 详细说明](docs/packages/hand_detector.md)
- [`pc_processor` 详细说明](docs/packages/pc_processor.md)
- [`handover_task` 详细说明](docs/packages/handover_task.md)
- [`handover_task_interfaces` 详细说明](docs/packages/handover_task_interfaces.md)
- [`ur_robotiq` 详细说明](docs/packages/ur_robotiq.md)
- [`ur_robotiq_interfaces` 详细说明](docs/packages/ur_robotiq_interfaces.md)

## 现有问题说明

- 当前 YOLO 分割能力不足，现有模型开发较仓促，无法稳定剔除整条手臂。（新用了200张数据集做了个专门剔除手臂的模型，未测试）
- 目前抓取旋转角与抓取宽度无法可靠估计，抓稳主要依赖写死的力控参数与闭合宽度。（在做触觉了）
- 状态估计相关参数仍需持续调参（含滤波与阈值类参数），才能达到更稳定的实际效果。（建议使用加速度状态转移模型，并换成fillter库，对瞬时运动有更好的效果）
