# UR5 RTDE 伺服路径追踪设计说明

## 1. 目标

基于 `ur_rtde` 编写一个极简 ROS2 Python 包，用于 UR5 路径追踪。  
节点订阅 `geometry_msgs/msg/PoseStamped`，支持 20 Hz 及以上输入，并通过 `servoL` 执行完整 6D 位姿跟踪（位置 + 姿态）。

## 2. 范围

### 包含内容（In Scope）

- 仅保留一个 ROS2 节点类：`RTDEServoNode`
- 输入消息类型：`PoseStamped`
- 坐标系解释：以消息 `msg.header.frame_id` 为准，并进行运行时校验
- 缓冲策略：latest-only（新目标覆盖旧目标，不排队）
- 控制循环默认频率：`20 Hz`
- 跟踪维度：完整 6D（位置 + 姿态）
- 最小文件结构：一个核心 Python 文件 + 一个详细 YAML 配置文件
- launch 仅用于常用重映射和常用参数入口
- 详细参数调优只能通过 launch 指定的 YAML 配置文件完成
- 使用工作区 `.venv`，并在 `.venv/bin/activate` 中加入 `PYTHONPATH` 导出命令

### 不包含内容（Out of Scope）

- 多节点控制器拆分
- Action 接口
- 多控制策略规划器
- 过度复杂的参数分层体系

## 3. 架构设计

采用单节点架构，强调高内聚、低耦合。

- 核心类：`RTDEServoNode`
- 运行时逻辑全部内聚在该类中
- 主数据流函数不加 `_` 前缀
- 内部辅助流程函数使用 `_` 前缀

建议的最小文件布局：

- `ur5_pose_tracker/pose_tracker_node.py`
- `config/pose_tracker.yaml`
- `launch/pose_tracker.launch.py`（仅入口，不展开详细参数）

## 4. 节点内部设计（`RTDEServoNode`）

### 主数据流函数

- `run_control_loop`：定时器驱动的主控制流程，默认 20 Hz

### 内部辅助函数

- `_on_pose_msg`：接收位姿消息，校验坐标系，并覆盖写入最新目标
- `_validate_frame`：检查 `header.frame_id` 是否在允许列表中
- `_pose_to_servol_target`：将 ROS 位姿转换为 RTDE `servoL` 所需 6D 目标
- `_send_servo`：按配置参数调用 `servoL`
- `_safe_stop`：超时、异常或退出时安全停止并释放 RTDE 资源

## 5. 数据流与时序

1. 订阅器接收 `PoseStamped`
2. `_on_pose_msg` 校验 `frame_id` 并更新最新位姿缓存
3. 20 Hz 定时器触发 `run_control_loop`
4. `run_control_loop` 读取当前最新位姿（若存在）
5. 转换为 6D 目标并发送 `servoL`
6. 循环执行，始终追踪“最新目标”

行为规则：

- 输入频率可高于 20 Hz，但只使用最新目标（latest-only）
- 当没有有效目标时，本周期安全跳过控制输出
- 位姿流超时后按超时策略执行 `_safe_stop`

## 6. 参数与 Launch 约束

所有详细参数集中放在 `config/pose_tracker.yaml`，包括但不限于：

- `robot_ip`
- `input_topic`
- `control_hz`（默认 20）
- `accepted_frame_ids`
- `pose_timeout_sec`
- `servo_*` 调优参数（如 `speed`、`acceleration`、`lookahead_time`、`gain` 等）

launch 约束：

- `launch/pose_tracker.launch.py` 仅处理：
  - 常用话题重映射
  - 常用顶层参数覆盖
- 详细调优参数不在 launch 中重复声明，必须在 YAML 中维护

## 7. 错误处理与日志

日志风格：英文、简洁明了。

- RTDE 连接失败：记录 error 并启动失败退出
- 坐标系不匹配：丢弃该消息并记录 warn
- `servoL` 运行异常：记录 error 并执行 `_safe_stop`
- 位姿流超时：记录 warn 并停止伺服
- 节点退出：始终清理 RTDE 连接资源

## 8. 最小验证方案

- 节点可通过 YAML 配置正常启动
- 20 Hz `PoseStamped` 输入可持续被消费
- 错误 `frame_id` 会被拒绝并产生告警日志
- 消息流超时可触发安全停止
- Ctrl+C 退出后 RTDE 连接可正确释放

## 9. 环境约束

运行环境使用工作区 `.venv`。  
在 `.venv/bin/activate` 中加入导出命令，使包路径可通过 `PYTHONPATH` 被正确解析。

