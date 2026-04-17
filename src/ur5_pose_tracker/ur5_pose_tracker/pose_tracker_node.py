import math
import threading
import time
from typing import Optional

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
except ImportError:  # pragma: no cover - fallback for test environment
    rclpy = None
    PoseStamped = object
    Node = object

try:
    from rtde_control import RTDEControlInterface
    from rtde_receive import RTDEReceiveInterface
except ImportError:  # pragma: no cover - fallback for test environment
    RTDEControlInterface = None
    RTDEReceiveInterface = None


class _SimpleLogger:
    def info(self, message):
        print(message)

    def warning(self, message):
        print(message)

    def error(self, message):
        print(message)


class RTDEServoNode:
    def __init__(
        self,
        robot_ip="127.0.0.1",
        expected_frame_id="base",
        control_hz=20.0,
        speed=0.25,
        acceleration=1.2,
        lookahead_time=0.1,
        gain=300.0,
        rtde_control=None,
        rtde_receive=None,
        logger=None,
        use_ros=True,
    ):
        self._expected_frame_id = expected_frame_id
        self._control_hz = float(control_hz)
        self._speed = float(speed)
        self._acceleration = float(acceleration)
        self._lookahead_time = float(lookahead_time)
        self._gain = float(gain)
        self._dt = 1.0 / self._control_hz
        self._latest_target = None
        self._target_lock = threading.Lock()
        self._running = True

        self._ros_node = None
        self._logger = logger or _SimpleLogger()
        if use_ros:
            if rclpy is None:
                raise RuntimeError("rclpy is required when use_ros=True")
            self._ros_node = Node("rtde_servo_node")
            self._logger = self._ros_node.get_logger()
            self._ros_node.create_subscription(
                PoseStamped,
                "target_pose",
                self._on_pose_msg,
                10,
            )

        self._rtde_control = rtde_control
        if self._rtde_control is None:
            if RTDEControlInterface is None:
                raise RuntimeError("ur_rtde control interface is not available")
            self._rtde_control = RTDEControlInterface(robot_ip)

        self._rtde_receive = rtde_receive
        if self._rtde_receive is None:
            if RTDEReceiveInterface is None:
                raise RuntimeError("ur_rtde receive interface is not available")
            self._rtde_receive = RTDEReceiveInterface(robot_ip)

    def _on_pose_msg(self, msg):
        if not self._validate_frame(msg.header.frame_id):
            self._logger.warning("Ignore pose with invalid frame.")
            return

        target = self._pose_to_servol_target(msg.pose)
        with self._target_lock:
            self._latest_target = target

    def _validate_frame(self, frame_id):
        return bool(frame_id) and frame_id == self._expected_frame_id

    def _pose_to_servol_target(self, pose):
        qx = float(pose.orientation.x)
        qy = float(pose.orientation.y)
        qz = float(pose.orientation.z)
        qw = float(pose.orientation.w)
        rx, ry, rz = self._quat_to_rotvec(qx, qy, qz, qw)
        return [
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
            rx,
            ry,
            rz,
        ]

    def _send_servo(self, target):
        self._rtde_control.servoL(
            target,
            self._speed,
            self._acceleration,
            self._dt,
            self._lookahead_time,
            self._gain,
        )

    def _safe_stop(self):
        try:
            self._rtde_control.servoStop()
        except Exception as exc:  # pragma: no cover - hardware safety path
            self._logger.error(f"servoStop failed: {exc}")

    def run_control_loop(self, max_steps=None):
        step = 0
        try:
            while self._running:
                with self._target_lock:
                    target = self._latest_target

                if target is not None:
                    self._send_servo(target)

                # 主控制数据流 Main control data flow: spin once and send latest target.
                if self._ros_node is not None and rclpy is not None:
                    rclpy.spin_once(self._ros_node, timeout_sec=0.0)
                    if not rclpy.ok():
                        break

                step += 1
                if max_steps is not None and step >= max_steps:
                    break
                time.sleep(self._dt)
        finally:
            self._safe_stop()

    @staticmethod
    def _quat_to_rotvec(qx, qy, qz, qw):
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm == 0.0:
            return 0.0, 0.0, 0.0
        qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

        angle = 2.0 * math.acos(max(-1.0, min(1.0, qw)))
        sin_half = math.sqrt(max(0.0, 1.0 - qw * qw))
        if sin_half < 1e-9:
            return 0.0, 0.0, 0.0
        axis_x = qx / sin_half
        axis_y = qy / sin_half
        axis_z = qz / sin_half
        return axis_x * angle, axis_y * angle, axis_z * angle


def main(args: Optional[list] = None):
    if rclpy is None:
        raise RuntimeError("rclpy is required to run RTDEServoNode")

    rclpy.init(args=args)
    node = RTDEServoNode(use_ros=True)
    try:
        node.run_control_loop()
    except KeyboardInterrupt:
        node._logger.info("Shutdown requested.")
    finally:
        if node._ros_node is not None:
            node._ros_node.destroy_node()
        rclpy.shutdown()
