# UR5 RTDE Servo Tracking Design

## 1. Goal

Build a minimal ROS2 Python package for UR5 path tracking using `ur_rtde`.
The node subscribes to `geometry_msgs/msg/PoseStamped` at 20 Hz or higher and performs full 6D pose tracking with `servoL`.

## 2. Scope

### In Scope

- One ROS2 node class: `RTDEServoNode`
- Input message type: `PoseStamped`
- Frame interpretation: follow `msg.header.frame_id`, with runtime validation
- Buffer policy: latest-only (overwrite older target poses)
- Control loop default: `20 Hz`
- Full 6D tracking: position + orientation
- Minimal layout: one core Python file + one detailed YAML config
- Launch only exposes common remap and common argument overrides
- Detailed tuning must be done via config YAML passed from launch
- Use `.venv` and add `PYTHONPATH` export command in `.venv/bin/activate`

### Out of Scope

- Multi-node controller decomposition
- Action interface
- Multi-strategy control planners
- Over-engineered parameter layers

## 3. Architecture

Use a single-node architecture with high cohesion and low coupling.

- Core class: `RTDEServoNode`
- All runtime logic stays inside this node
- Main dataflow methods do not use underscore prefix
- Internal helper flows use underscore prefix

Proposed minimal file layout:

- `ur5_pose_tracker/pose_tracker_node.py`
- `config/pose_tracker.yaml`
- `launch/pose_tracker.launch.py` (entry only, no detailed parameter expansion)

## 4. Node Internal Design (`RTDEServoNode`)

### Main Dataflow Method(s)

- `run_control_loop`: timer-based control path, default 20 Hz

### Internal Helper Method(s)

- `_on_pose_msg`: receive pose, validate frame, overwrite latest target
- `_validate_frame`: check `header.frame_id` against configured accepted frames
- `_pose_to_servol_target`: convert ROS pose to RTDE servoL 6D target
- `_send_servo`: call `servoL` with configured control parameters
- `_safe_stop`: stop servo and release RTDE resources on timeout/error/shutdown

## 5. Data Flow and Timing

1. Subscriber receives `PoseStamped`
2. `_on_pose_msg` validates frame and updates latest pose cache
3. 20 Hz timer calls `run_control_loop`
4. `run_control_loop` reads current latest pose (if available)
5. Convert to 6D target and send `servoL`
6. Repeat, always tracking newest pose only

Behavioral rules:

- Input rate may exceed 20 Hz; only latest target is used
- If no valid pose exists, control tick is skipped safely
- If pose stream times out, invoke `_safe_stop` per timeout policy

## 6. Parameters and Launch Contract

All detailed parameters live in `config/pose_tracker.yaml`, including:

- `robot_ip`
- `input_topic`
- `control_hz` (default 20)
- `accepted_frame_ids`
- `pose_timeout_sec`
- `servo_*` tuning parameters (`speed`, `acceleration`, `lookahead_time`, `gain`, etc.)

Launch contract:

- `launch/pose_tracker.launch.py` can only handle:
  - common topic remap
  - common top-level argument overrides
- Detailed tuning is not duplicated in launch and must be edited in YAML

## 7. Error Handling and Logging

Logging style is simple English messages.

- RTDE connect fail: log error and fail startup
- Frame mismatch: drop message and warn
- `servoL` runtime exception: log error and `_safe_stop`
- Pose timeout: warn and stop servo
- Shutdown: always cleanup RTDE connection

## 8. Verification Plan (Minimal)

- Node can launch with config YAML
- 20 Hz `PoseStamped` input is consumed continuously
- Wrong `frame_id` is rejected with warning
- Stream timeout triggers safe stop
- Ctrl+C shutdown releases RTDE cleanly

## 9. Environment Constraint

Use workspace `.venv` for runtime.
In `.venv/bin/activate`, add export command so package path is available through `PYTHONPATH`.

