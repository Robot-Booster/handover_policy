# pc_processor

`pc_processor` is a single-node ROS 2 (`ament_cmake`) package that runs the full depth-to-held-object pipeline in one OpenMP-accelerated process.

## Pipeline

Per-frame main data flow:

1. **Fused projection** (OpenMP parallel): `depth(32FC1) + seg_mask(mono8) + roi_mask(static PNG) + OBB workspace` → compact `Point[]` with `is_target` flag.
2. **Voxel hash** (single pass): accumulate per-voxel `sum_xyz / count / target_points`.
3. **ROR (voxel-level)**: drop voxels with < `ror_min_neighbors` neighbors in 26-connectivity.
4. **Cluster (voxel-level)**: 26-connected Union-Find on target voxels and other voxels separately; filter by `cluster_min_voxels`.
5. **Adjacency + selection**: for each target cluster, find its adjacent other clusters; globally pick the single biggest adjacent other cluster as the final held object.
6. **TF** to `target_frame`, **paint** uniform `output_rgb`, **publish**:
   - `cloud_topic` as `sensor_msgs/PointCloud2 (xyzrgb)`
   - `centroid_topic` as `geometry_msgs/PointStamped` (weighted centroid)

Frames where nothing is selected publish neither topic.

## Parameters (YAML only unless noted)

Topics (CLI-overridable via launch):
- `depth_topic`, `depth_camera_info_topic`, `seg_mask_topic`, `cloud_topic`, `centroid_topic`

Frames (CLI-overridable):
- `target_frame` (empty → stay in depth frame)

Static resources (YAML only):
- `roi_mask_image_path`
- `workspace_box_json_path` (`pc_mask_box.json` schema)

Sync (YAML only):
- `depth_mask_max_stamp_diff_sec` (default `0.05`)
- `allow_future_mask` (default `false`)

Filtering (YAML only):
- `voxel_leaf_size_m` (default `0.005`)
- `ror_min_neighbors` (default `5`, max `26`)
- `cluster_min_voxels` (default `20`)
- `output_rgb` (3× uint8, default `[255, 0, 0]`)

## Launch

```bash
ros2 launch pc_processor perception.launch.py \
  namespace:=d455 \
  config_file:=/home/ender/handover_baseline/src/pc_processor/config/d455_perception.yaml
```

Launch arguments (empty string = keep YAML value):

- `config_file`, `namespace`
- `depth_topic`, `depth_camera_info_topic`, `seg_mask_topic`
- `cloud_topic`, `centroid_topic`
- `target_frame`

All filter/sync/static-resource parameters are YAML-only; CLI cannot change them.
