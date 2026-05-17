# cam_pre 详细说明

## 文件结构

- `src/camera_preprocess_node.cpp`：ROS 接口层（订阅、参数、发布）。
- `src/preprocess_pipeline.cpp`：去畸变流水线（`initUndistortRectifyMap` + `remap`，更新 rect `CameraInfo`）。
- `include/camera/camera_preprocess_node.hpp`：节点声明。
- `include/camera/preprocess_pipeline.hpp`：流水线声明。
- `launch/cam_preprocess.launch.py`：启动脚本。
- `config/config.yaml`：话题名配置。

## 数据流

1. 订阅原始 `image_topic` 与 `camera_info_topic`。
2. 缓存最新 `CameraInfo`；标定更新时重算 undistort map。
3. 图像回调内对当前帧做去畸变，发布 rect 图像与 rect `CameraInfo`。
4. 输出消息的 `header.stamp`、`frame_id` 与输入图像一致。

## 注意点

- launch 必须提供 `config_file`；`namespace` + `camera_name` 决定节点全名（如 `/left/d455`）。
- 标定未就绪时跳过本帧并打日志。
- 下游（`fastfoundation`、`hand_detector`）应订阅 rect 话题，而非 raw。

## 参数详解

- `image_topic`：输入原始图像（默认 `~/color/image_raw`）。
- `camera_info_topic`：输入相机内参（默认 `~/color/camera_info`）。
- `image_rect_topic`：输出去畸变图像（默认 `~/color/image_rect`）。
- `camera_info_rect_topic`：输出 rect 内参（默认 `~/color/camera_info_rect`）。
