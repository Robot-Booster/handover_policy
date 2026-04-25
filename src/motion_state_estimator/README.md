# motion_state_estimator

`motion_state_estimator` provides `grasp_pose_predictor_node`, which estimates target motion from point observations and outputs grasp poses with a locked orientation.

## Topic I/O

- Subscriptions:
  - `input_point_topic` (default `~/target_point`): `geometry_msgs/msg/PointStamped`
  - `input_tcp_pose_topic` (default `~/tcp_pose`): `geometry_msgs/msg/PoseStamped`
- Publications:
  - `output_grasp_pose_topic` (default `~/grasp_pose`): `geometry_msgs/msg/PoseStamped`
  - `output_debug_point_topic` (default `~/debug_point`): `geometry_msgs/msg/PointStamped` (enabled by `enable_debug_point`)
  - `workspace_marker_topic` (default `~/workspace_marker`): `visualization_msgs/msg/Marker` (`CUBE`)

The workspace marker is generated from `workspace_box_json_path` and published in `base_frame` with color `workspace_marker_rgba` (default `[0.0, 0.0, 1.0, 0.25]`).

## Required TF Chain

The node resolves `base_frame -> ee_frame` to get the current TCP local +Z direction for offset application. This TF chain must be available at runtime.

## Launch

```bash
ros2 launch motion_state_estimator grasp_pose_predictor.launch.py \
  config_file:=/abs/path/to/grasp_pose_predictor.yaml \
  input_point_topic:=/camera/target_point \
  input_tcp_pose_topic:=/robot/tcp_pose \
  output_grasp_pose_topic:=/robot/grasp_pose \
  workspace_marker_topic:=/robot/workspace_marker
```

Notes:
- Topic CLI arguments are optional overrides.
- If a topic CLI value is empty (or whitespace-only), launch keeps the YAML value unchanged.
