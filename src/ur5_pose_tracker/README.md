# ur5_pose_tracker

## Quick start

```bash
source .venv/bin/activate
```

```bash
ros2 launch ur5_pose_tracker pose_tracker.launch.py params_file:=src/ur5_pose_tracker/config/pose_tracker.yaml
```

## Publish target pose at 20 Hz

```bash
ros2 topic pub /target_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'base'}, pose: {position: {x: 0.40, y: -0.10, z: 0.35}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}" -r 20
```
