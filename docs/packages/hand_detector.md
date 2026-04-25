# hand_detector 详细说明

## 文件结构

- `hand_detector/hand_detector_node.py`：分割主节点，YOLO 推理与掩码发布核心逻辑。
- `launch/hand_detector.launch.py`：启动脚本，负责参数装配。
- `config/config.yaml`：默认参数配置（模型路径、类别、置信度等）。
- `weights/`：模型权重目录占位（实际权重由你自行放置）。

## 数据流

1. 订阅 `~/image_raw` 图像，转为 OpenCV BGR。
2. 读取 `model_path` 加载 YOLO 分割模型，解析 `label` 对应的 `class_id`。
3. 每帧执行 `model.predict()`，筛选目标类别实例并做 mask 并集。
4. 输出 `mono8` 二值掩码到 `~/hand_mask`，供 `pc_processor` 融合过滤。

## 注意点

- `model_path` 必须是绝对路径且文件存在，否则节点启动失败。
- `device` 设为 CUDA 时会检查 `torch.cuda.is_available()`，不满足会直接报错。
- 当前策略是同一类别“所有实例并集”，不会区分左右手或单独实例 ID。
- 按当前项目问题，模型对整条手臂剔除能力不足，需专门训练“手臂+手掌”分割模型。

## 参数详解

- `model_path`：分割模型权重绝对路径（必填）。
- `label`：目标类别名（需在模型类别表中存在）。
- `device`：推理设备（如 `cuda:0`、`cpu`）。
- `confidence`：置信度阈值。
- `imgsz`：推理输入尺寸。
- `max_det`：单帧最大检测实例数。
- `retina_masks`：是否启用高质量 mask 输出。
