# handover_task 详细说明

## 文件结构

- `handover_task/base_policy.py`：单节点主流程（卡尔曼跟踪、工作区约束、抓取位姿发布、任务状态机）。
- `handover_task/linear_cv_kalman.py`：三轴独立线性 CV 卡尔曼滤波。
- `launch/base_policy.launch.py`：启动脚本与话题 remap。
- `config/base_policy.yaml`：策略参数。
- `config/pc_mask_box.json`：工作空间 OBB 约束。

## 数据流

1. 订阅目标质心 `input_point_topic` 与 TCP 位姿 `input_tcp_pose_topic`。
2. 卡尔曼滤波 + OBB 约束，持续发布抓取目标位姿到 `output_grasp_pose_topic`（控制使能时）。
3. 服务 `~/base_policy` 触发一次任务：`追踪 → 抓取 → 交接 → 回初始`。
4. 追踪阶段用距离容差 + 稳定时长判定进入抓取。
5. 抓取阶段沿 TCP `Z+` 前伸 `tcp_z_approach_m`，调用 `move_l_service` 与 `set_gripper_service`。
6. 返回阶段按 `use_custom_return_pose` 回自定义位姿或初始位姿；结束打开夹爪。

## ROS 接口

| 方向 | 参数/逻辑名 | 类型 |
|------|-------------|------|
| Sub | `input_point_topic` | `geometry_msgs/PointStamped` |
| Sub | `input_tcp_pose_topic` | `geometry_msgs/PoseStamped` |
| Pub | `output_grasp_pose_topic` | `geometry_msgs/PoseStamped` |
| Pub | `output_estimated_point_topic` | `geometry_msgs/PointStamped` |
| Pub | `workspace_marker_topic` | `visualization_msgs/Marker` |
| Srv | `~/base_policy` | `handover_task_interfaces/srv/BasePolicy` |
| Client | `move_l_service` | `ur_robotiq_interfaces/srv/MoveToPose` |
| Client | `set_gripper_service` | `ur_robotiq_interfaces/srv/SetGripper` |

## 注意点

- `task_timeout_sec` 超时返回 `FAIL_TIMEOUT`。
- `custom_return_pose` 为 `position` + `orientation_xyzw` 同构参数。
- 失败码详见 [`handover_task_interfaces`](handover_task_interfaces.md)。
- launch 可通过 CLI 覆盖话题 remap，未指定则用 YAML 绝对路径。

## 参数详解

- `input_point_topic` / `input_tcp_pose_topic` / `output_grasp_pose_topic`：上下游话题。
- `output_frame_id`：输出位姿 frame（如 `base_link`）。
- `update_hz`：控制循环频率。
- `process_noise_pos` / `process_noise_vel` / `measure_noise_pos`：卡尔曼噪声。
- `feedforward_sec`：前馈预测时长。
- `lost_timeout_sec`：目标丢失超时。
- `tcp_z_offset_m` / `tcp_z_approach_m`：抓取相对 TCP 的 Z 偏移与前伸距离。
- `grasp_distance_tolerance_m` / `grasp_stable_duration_sec`：进入抓取的稳定判定。
- `task_timeout_sec`：整次任务超时。
- `use_custom_return_pose` / `custom_return_pose`：交接返回位姿。
- `close_gripper_position` / `open_gripper_position`：夹爪开合位置 [0,1]。
- `move_l_service` / `set_gripper_service`：机械臂服务绝对路径。
- `workspace_box_json_path` / `workspace_marker_rgba`：工作区 OBB 与可视化。
