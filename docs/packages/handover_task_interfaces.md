# handover_task_interfaces 详细说明

## 文件结构

- `srv/BasePolicy.srv`：任务编排服务定义。
- `CMakeLists.txt`、`package.xml`：接口包构建与导出配置。

## 服务定义

`BasePolicy.srv`：

- 请求：`bool trigger`
- 响应：
  - `bool success`
  - `string code`
  - `string message`

## 失败码约定（与 `handover_task` 实现一致）

| code | 含义 |
|------|------|
| `OK` | 成功 |
| `FAIL_BUSY` | 已有任务在执行 |
| `FAIL_TIMEOUT` | 追踪/任务超时 |
| `FAIL_GRASP_MOVE` | 抓取位姿不可用或 moveL 失败 |
| `FAIL_GRIPPER` | 夹爪控制失败 |
| `FAIL_HANDOVER_POSE` | 交接位姿无效或 moveL 失败 |
| `FAIL_RETURN_INITIAL` | 回初始位姿失败 |
