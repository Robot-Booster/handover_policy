# pc_processor

`pc_processor` is a ROS 2 `ament_cmake` C++ package that converts depth image (`32FC1`) + `CameraInfo` into `xyzrgb` point cloud.

## Pipeline

Main data flow:

1. `roi`: apply optional fixed `mono8` roi image on depth (`0` invalid, `>0` keep).
2. `project`: depth + camera intrinsics -> xyz.
3. `align`: transform xyz to `align_frame` via tf2 when `align_frame` is not empty.
4. `mask`: apply optional `mono8` mask topic.
5. `colorize`: generate depth heatmap rgb.
6. `publish`: publish `sensor_msgs/msg/PointCloud2` (`xyzrgb`).

If mask frame is not equal to final cloud frame, node prints warning and skips mask for that frame only.

## Parameters

- `depth_topic` (string, default: `~/depth`)
- `depth_camera_info_topic` (string, default: `~/depth_camera_info`)
- `mask_topic` (string, default: ``)
- `mask_camera_info_topic` (string, default: ``, enable mask reprojection when set)
- `undistort_mask_enabled` (bool, default: `true`, yaml-only)
  - If enabled, `mask_camera_info_topic` must not be empty.
  - Only `plumb_bob` distortion model is supported.
  - If mask camera distortion coefficients are all zero, node prints one warning.
- `cloud_topic` (string, default: `~/pointcloud`)
- `align_frame` (string, default: ``)
- `roi_mask_image_path` (string, default: ``, yaml-only)
- `heatmap_min_depth_m` (double, default: `0.1`)
- `heatmap_max_depth_m` (double, default: `2.0`)

Node prints all parameter values with `INFO` log at startup.

## Launch

Use:

```bash
ros2 launch pc_processor depth_to_cloud.launch.py
```

Launch file supports overriding only:

- `config_file`
- `namespace`
- `depth_topic`
- `depth_camera_info_topic`
- `mask_topic`
- `mask_camera_info_topic`
- `cloud_topic`
- `align_frame`

Heatmap range is configured in yaml and not exposed for launch override.
ROI image path is configured in yaml and not exposed for launch override.
