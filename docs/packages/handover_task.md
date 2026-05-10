# handover_task 详细说明

## 文件结构

- `handover_task/base_policy.py`：单节点主流程（状态预测、追踪、抓取、返回）。
- `handover_task/linear_cv_kalman.py`：线性 CV 卡尔曼滤波器。
- `launch/base_policy.launch.py`：启动脚本与参数入口。
- `config/base_policy.yaml`：策略参数配置。
- `config/pc_mask_box.json`：工作空间 OBB 约束配置。

## 数据流

1. 订阅目标点和 TCP 位姿，持续发布抓取目标位姿到 `output_grasp_pose_topic`。
2. 服务 `~/base_policy` 被调用后，按 `追踪 -> 抓取 -> 返回 -> 回初始` 执行一次任务。
3. 追踪阶段使用距离容差 + 稳定时长判定进入抓取。
4. 抓取阶段沿 TCP `Z+` 前伸 `tcp_z_approach_m`，然后闭合夹爪。
5. 返回阶段按 `use_custom_return_pose` 决定回自定义位姿或初始位姿。
6. 任务结束统一回初始位姿并打开夹爪，等待下一次调用。

## 注意点

- `task_timeout_sec` 超时返回 `FAIL_TIMEOUT`。
- 目标丢失判定复用 `lost_timeout_sec`，返回 `FAIL_TARGET_LOST`。
- `custom_return_pose` 使用 `PoseStamped` 同构参数：`position + orientation_xyzw`。
