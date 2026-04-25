# ur_robotiq

## Quick start

```bash
source .venv/bin/activate
```

```bash
ros2 launch ur_robotiq pose_tracker.launch.py params_file:=src/ur_robotiq/config/pose_tracker.yaml
```

## Publish target pose at 20 Hz

```bash
ros2 topic pub /target_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'base'}, pose: {position: {x: 0.40, y: -0.10, z: 0.35}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}" -r 20
```

## Services

```bash
ros2 service call /ur_robotiq/move_l ur_robotiq_interfaces/srv/MoveToPose "{target: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.4, y: 0.0, z: 0.35}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
```

```bash
ros2 service call /ur_robotiq/set_gripper ur_robotiq_interfaces/srv/SetGripper "{position: 0.3}"
```
