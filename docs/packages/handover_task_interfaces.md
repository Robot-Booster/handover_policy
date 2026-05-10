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

## 失败码约定

- `OK`
- `FAIL_BUSY`
- `FAIL_TIMEOUT`
- `FAIL_TARGET_LOST`
- `FAIL_GRASP_MOVE`
- `FAIL_GRIPPER`
- `FAIL_RETURN_POSE`
- `FAIL_RETURN_INITIAL`
