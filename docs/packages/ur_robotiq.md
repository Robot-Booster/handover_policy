# ur_robotiq 详细说明

## 文件结构

- `ur_robotiq/pose_tracker_node.py`：UR RTDE 控制主节点（servo、moveL、夹爪、TF、TCP 发布）。
- `launch/pose_tracker.launch.py`：启动脚本与参数入口。
- `config/pose_tracker.yaml`：机器人 IP、控制频率、手眼参数、夹爪参数等配置。

## 数据流

1. 订阅目标位姿 `input_topic`（默认 `~/ur_target_pose`）。
2. 校验 `frame_id` 后，将姿态四元数转为 UR 旋转向量目标。
3. 控制循环中：
   - 普通跟踪路径走 `servoL`；
   - 服务调用路径走 `moveL`（阻塞执行）。
4. 周期读取 RTDE 实际 TCP 位姿，发布到 `~/tcp_pose`。
5. 同步发布 `base_frame -> ee_frame` 动态 TF；可选发布手眼静态 TF。
6. 提供夹爪服务（若串口与依赖可用）。

## 注意点

- 机械臂对接时，建议保持上层接口语义与 `ur_robotiq_interfaces` 一致，便于替换底层驱动。
- `servoL` 对 `speed/acceleration` 的实时意义有限，核心调参常在 `control_hz`、`lookahead_time`、`gain`。
- `accepted_frame_ids` 必须与上游输出 frame 对齐，否则目标会被直接丢弃。
- 夹爪控制依赖 `pyrobotiqgripper` 与串口设备，缺失时只禁用夹爪，不影响机械臂位姿控制。

## 参数详解

- `robot_ip`：UR 控制器 IP。
- `input_topic`：目标位姿输入话题（`PoseStamped`）。
- `control_hz`：控制循环频率。
- `accepted_frame_ids`：允许接收的输入 frame 白名单。
- `pose_timeout_sec`：目标超时阈值，超时后停止伺服。
- `servo_speed`：`servoL` 速度参数（实时影响有限）。
- `servo_acceleration`：`servoL` 加速度参数（主要用于 `servoStop` 减速）。
- `servo_lookahead_time`：`servoL` 前瞻时间。
- `servo_gain`：`servoL` 伺服增益。
- `move_l_speed`：`moveL` 速度。
- `move_l_acceleration`：`moveL` 加速度。
- `gripper_com_port`：夹爪串口设备路径。
- `gripper_speed`：夹爪动作速度（0-255）。
- `gripper_force`：夹爪动作力度（0-255）。
- `handeye_method`：手眼模式（如 `eye_on_hand`、`eye_on_base`）。
- `base_frame`：机器人基坐标系。
- `ee_frame`：末端执行器坐标系。
- `camera_frame`：相机坐标系。
- `handeye_tf.translation`：手眼平移向量 `[x,y,z]`。
- `handeye_tf.rotation`：手眼旋转矩阵（按行展开 9 元素）。
