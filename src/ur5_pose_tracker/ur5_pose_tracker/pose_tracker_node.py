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
        expected_frame_id=None,
        accepted_frame_ids=None,
        input_topic="/target_pose",
        control_hz=20.0,
        speed=0.25,
        acceleration=1.2,
        lookahead_time=0.1,
        gain=300.0,
        pose_timeout_sec=0.5,
        rtde_control=None,
        rtde_receive=None,
        logger=None,
        use_ros=True,
    ):
        if accepted_frame_ids is None:
            accepted_frame_ids = [expected_frame_id or "base"]

        self._robot_ip = str(robot_ip)
        self._input_topic = str(input_topic)
        self._accepted_frame_ids = {str(frame) for frame in accepted_frame_ids if frame}
        if not self._accepted_frame_ids:
            self._accepted_frame_ids = {"base"}
        self._control_hz = float(control_hz)
        self._speed = float(speed)
        self._acceleration = float(acceleration)
        self._lookahead_time = float(lookahead_time)
        self._gain = float(gain)
        self._pose_timeout_sec = float(pose_timeout_sec)
        self._dt = 1.0 / self._control_hz
        self._latest_target = None
        self._latest_target_time = 0.0
        self._target_lock = threading.Lock()
        self._running = True

        self._ros_node = None
        self._logger = logger or _SimpleLogger()
        if use_ros:
            if rclpy is None:
                raise RuntimeError("rclpy is required when use_ros=True")
            self._ros_node = Node("rtde_servo_node")
            self._logger = self._ros_node.get_logger()
            self._declare_ros_parameters()
            self._load_ros_parameters()
            self._ros_node.create_subscription(
                PoseStamped,
                self._input_topic,
                self._on_pose_msg,
                10,
            )

        self._rtde_control = rtde_control
        if self._rtde_control is None:
            if RTDEControlInterface is None:
                raise RuntimeError("ur_rtde control interface is not available")
            self._rtde_control = RTDEControlInterface(self._robot_ip)

        self._rtde_receive = rtde_receive
        if self._rtde_receive is None:
            if RTDEReceiveInterface is None:
                raise RuntimeError("ur_rtde receive interface is not available")
            self._rtde_receive = RTDEReceiveInterface(self._robot_ip)

    def _declare_ros_parameters(self):
        self._ros_node.declare_parameter("robot_ip", self._robot_ip)
        self._ros_node.declare_parameter("input_topic", self._input_topic)
        self._ros_node.declare_parameter("control_hz", self._control_hz)
        self._ros_node.declare_parameter(
            "accepted_frame_ids", sorted(self._accepted_frame_ids)
        )
        self._ros_node.declare_parameter("pose_timeout_sec", self._pose_timeout_sec)
        self._ros_node.declare_parameter("servo_speed", self._speed)
        self._ros_node.declare_parameter("servo_acceleration", self._acceleration)
        self._ros_node.declare_parameter("servo_lookahead_time", self._lookahead_time)
        self._ros_node.declare_parameter("servo_gain", self._gain)

    def _load_ros_parameters(self):
        self._robot_ip = str(self._ros_node.get_parameter("robot_ip").value)
        self._input_topic = str(self._ros_node.get_parameter("input_topic").value)
        self._control_hz = float(self._ros_node.get_parameter("control_hz").value)
        frames = self._ros_node.get_parameter("accepted_frame_ids").value
        self._accepted_frame_ids = {str(frame) for frame in frames if frame}
        if not self._accepted_frame_ids:
            self._accepted_frame_ids = {"base"}
        self._pose_timeout_sec = float(
            self._ros_node.get_parameter("pose_timeout_sec").value
        )
        self._speed = float(self._ros_node.get_parameter("servo_speed").value)
        self._acceleration = float(
            self._ros_node.get_parameter("servo_acceleration").value
        )
        self._lookahead_time = float(
            self._ros_node.get_parameter("servo_lookahead_time").value
        )
        self._gain = float(self._ros_node.get_parameter("servo_gain").value)
        self._dt = 1.0 / self._control_hz

    def _on_pose_msg(self, msg):
        if not self._validate_frame(msg.header.frame_id):
            self._logger.warning(f"Ignore pose with invalid frame: {msg.header.frame_id}")
            return

        target = self._pose_to_servol_target(msg.pose)
        with self._target_lock:
            self._latest_target = target
            self._latest_target_time = time.monotonic()

    def _validate_frame(self, frame_id):
        return bool(frame_id) and frame_id in self._accepted_frame_ids

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
                    target_time = self._latest_target_time

                if target is not None:
                    if (time.monotonic() - target_time) > self._pose_timeout_sec:
                        self._logger.warning("Pose timeout, stopping servo.")
                        self._safe_stop()
                        with self._target_lock:
                            self._latest_target = None
                            self._latest_target_time = 0.0
                    else:
                        try:
                            self._send_servo(target)
                        except Exception as exc:
                            self._logger.error(f"servoL failed: {exc}")
                            self._safe_stop()
                            with self._target_lock:
                                self._latest_target = None
                                self._latest_target_time = 0.0

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
