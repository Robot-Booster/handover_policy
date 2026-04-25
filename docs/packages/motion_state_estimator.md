# motion_state_estimator 详细说明

## 文件结构

- `motion_state_estimator/grasp_pose_predictor_node.py`：状态估计与抓取位姿输出主逻辑。
- `launch/grasp_pose_predictor.launch.py`：启动脚本与参数入口。
- `config/grasp_pose_predictor.yaml`：滤波与行为参数配置。
- `config/pc_mask_box.json`：工作空间 OBB 约束配置。

## 数据流

1. 订阅 `input_point_topic`（目标点）和 `input_tcp_pose_topic`（TCP 位姿）。
2. 首次 TCP 消息到来后锁定当前姿态（仅更新位置，不更新姿态）。
3. 定时器循环中对目标点做线性卡尔曼预测/更新。
4. 按 `feedforward_sec` 做前馈位置补偿。
5. 通过 `base_frame -> ee_frame` TF 计算末端 Z 方向，应用 `tcp_z_offset_m`。
6. 用工作空间 OBB 做 clamp，发布 `output_grasp_pose_topic` 与可选调试点。
7. 同步发布工作空间 Marker 便于可视化。

## 注意点

- 当前实现是“姿态锁定”策略，无法估计抓取旋转与抓取宽度。
- 对 TF 链依赖较强：`base_frame -> ee_frame` 缺失会导致当周期跳过发布。
- 跟踪稳定性主要受 `process_noise_*`、`measure_noise_pos`、`feedforward_sec`、`lost_timeout_sec` 影响，需要现场调参。
- `workspace_box_json_path` 是必填，且欧拉顺序要求 `ZYX`。

## 参数详解

- `input_point_topic`：输入目标点话题。
- `input_tcp_pose_topic`：输入 TCP 位姿话题。
- `output_grasp_pose_topic`：输出抓取位姿话题。
- `output_debug_point_topic`：输出调试点话题。
- `workspace_marker_topic`：输出工作空间 marker 话题。
- `output_frame_id`：抓取位姿输出 frame。
- `base_frame`：机器人基坐标系。
- `ee_frame`：末端执行器坐标系。
- `update_hz`：状态估计定时循环频率。
- `process_noise_pos`：卡尔曼位置过程噪声。
- `process_noise_vel`：卡尔曼速度过程噪声。
- `measure_noise_pos`：卡尔曼观测噪声。
- `feedforward_sec`：速度前馈预测时长（秒）。
- `lost_timeout_sec`：目标丢失超时（秒）。
- `tcp_z_offset_m`：沿末端局部 Z 方向的位置偏置（米）。
- `enable_debug_point`：是否发布调试点。
- `workspace_box_json_path`：工作空间 OBB 配置路径（必填）。
- `workspace_marker_rgba`：工作空间 marker 颜色（`[R,G,B,A]`）。
