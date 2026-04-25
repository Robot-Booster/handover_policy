# fastfoundation 详细说明

## 文件结构

- `fastfoundation/fastfoundation.py`：ROS2 节点入口，完成参数读取、双目同步、深度发布。
- `fastfoundation/model_api/api.py`：推理后端封装，支持 `engine`（TensorRT）与 `pt`（PyTorch）。
- `launch/launch_fastfoundation.launch.py`：启动脚本与参数传递入口。
- `config/d455.yaml`、`config/d435i.yaml`：不同相机配置示例。
- `weight/`：权重与推理配置目录（占位，由你自行放置实际文件）。

## 数据流

1. 订阅左右目图像（`cam1_topic`、`cam2_topic`），用 `ApproximateTimeSynchronizer` 做近似时间同步。
2. 订阅副相机 `CameraInfo`，从投影矩阵 `P` 提取 `fx` 和 `baseline`。
3. 同步回调内调用 `model_api.create_predictor()` 返回的推理器，输出视差图。
4. 用 `depth = fx * baseline / disparity` 计算深度图，发布 `32FC1` 到 `depth_img_topic`。

## 注意点

- 第三方 Fast-FoundationStereo 需按官方说明自行部署；本包仅做 ROS2 封装。
- 权重文件需自行准备并放入 `weight_dir`，`engine`/`pt` 模式所需文件不同。
- 推荐按目标机器硬件环境做 TensorRT engine 优化与构建，不建议跨机直接复用 engine。
- 当前代码对输入分辨率有上限约束（不超过 `640x480`），超出会丢帧并报警。
- 若 `CameraInfo` 未就绪或 `fx` 无效，不会发布深度结果。

## 参数详解

- `cam1_topic`：左目图像话题。
- `cam2_topic`：右目图像话题。
- `cam2_camera_info_topic`：右目相机内参话题；为空时按 `cam2_topic` 自动推导 `.../camera_info`。
- `weight_dir`：权重与后端文件目录。
- `device`：推理设备（如 `cuda:0`、`cpu`）。
- `inference_backend`：后端类型，`engine`（TensorRT）或 `pt`（PyTorch）。
- `depth_img_topic`：深度输出话题名（`32FC1`）。
