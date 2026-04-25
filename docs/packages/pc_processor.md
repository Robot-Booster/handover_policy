# pc_processor 详细说明

## 文件结构

- `src/main.cpp`：节点主入口，初始化 `PerceptionNode` 并进入 `rclcpp::spin`。
- `src/perception_node.cpp`：ROS 接口层（订阅、参数、发布、TF、同步）。
- `src/pc_pipeline.cpp`：点云处理主流程实现（投影、滤波、聚类、选择）。
- `include/pc_processor/perception_node.hpp`：节点接口定义。
- `include/pc_processor/pc_pipeline.hpp`：算法流水线接口定义。
- `config/perception.yaml`、`config/d455_perception.yaml`：参数模板与相机实例配置。
- `config/pc_mask_box.json`：工作空间 OBB 约束配置。
- `launch/perception.launch.py`：启动脚本，支持少量 topic/frame CLI 覆盖。

## 数据流

1. 订阅深度图、相机内参、分割掩码（含时间戳对齐）。
2. 融合深度 + 分割 + ROI + 工作空间约束，投影生成候选点集。
3. 做体素哈希聚合，再做体素级 ROR 去噪。
4. 对目标类与非目标类体素分别做聚类，按阈值过滤小簇。
5. 计算目标簇与邻接非目标簇关系，选出最终“被抓物体”簇。
6. 可选 TF 到目标坐标系后，发布 `PointCloud2` 与 `PointStamped` 质心。

## 注意点

- 目前结果对分割质量高度敏感：上游掩码把手臂误入时，会直接污染目标选择。
- 启动脚本只对部分参数开放 CLI 覆盖，滤波/聚类阈值主要走 YAML。
- `workspace_box_json_path`、`roi_mask_image_path` 这类静态资源路径必须可用。
- 调参重点通常在 `voxel_leaf_size_m`、`ror_min_neighbors`、`cluster_min_voxels`。

## 参数详解

- `depth_topic`：输入深度图话题。
- `depth_camera_info_topic`：输入深度相机内参话题。
- `seg_mask_topic`：输入分割掩码话题（`mono8`）。
- `cloud_topic`：输出点云话题。
- `centroid_topic`：输出目标质心话题。
- `workspace_marker_topic`：输出工作空间可视化 marker 话题。
- `target_frame`：输出参考坐标系；为空则保持深度图 frame。
- `roi_mask_image_path`：ROI 掩码图片绝对路径。
- `workspace_box_json_path`：工作空间 OBB 配置绝对路径。
- `depth_mask_max_stamp_diff_sec`：深度与掩码最大允许时间差。
- `allow_future_mask`：是否允许使用时间戳超前的掩码。
- `voxel_leaf_size_m`：体素下采样边长（米）。
- `ror_min_neighbors`：ROR 最小邻居数阈值（26 邻域）。
- `cluster_min_voxels`：保留簇的最小体素数。
- `output_rgb`：输出点云颜色（`[R,G,B]`，0-255）。
- `workspace_marker_rgba`：工作空间 marker 颜色（`[R,G,B,A]`，0-1）。
