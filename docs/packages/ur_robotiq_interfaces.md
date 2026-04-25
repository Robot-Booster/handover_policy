# ur_robotiq_interfaces 详细说明

## 文件结构

- `srv/MoveToPose.srv`：机械臂位姿运动服务定义。
- `srv/SetGripper.srv`：夹爪开合服务定义。
- `CMakeLists.txt`、`package.xml`：接口包构建与导出配置。

## 数据流

1. 上层节点（如 `motion_state_estimator` 后续执行逻辑）可通过服务调用控制执行层。
2. `MoveToPose.srv`：
   - 请求：`geometry_msgs/PoseStamped target`
   - 响应：`success + message`
3. `SetGripper.srv`：
   - 请求：`float32 position`（通常约定为 `[0,1]`）
   - 响应：`success + message`

## 注意点

- 该包只定义接口，不包含控制逻辑；具体执行在 `ur_robotiq` 或你的机械臂适配包中实现。
- 若替换机械臂驱动，优先保持这两个服务语义不变，可减少上层改造成本。
- `PoseStamped.frame_id` 的语义必须在控制实现侧严格校验并统一约定。

## 参数详解

该包为接口定义包，不包含 ROS 参数。  
参数配置在服务实现侧（例如 `ur_robotiq`）管理；调用侧只需按 `.srv` 结构构造请求。
