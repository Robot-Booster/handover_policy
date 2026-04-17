import math
import logging
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


class RTDEServoNode(Node):
    def __init__(
        self,
        robot_ip="127.0.0.1",
        expected_frame_id=None,
        accepted_frame_ids=None,
        input_topic="~/ur_target_pose",
        control_hz=20.0,
        speed=0.6,
        acceleration=0.3,
        lookahead_time=0.1,
        gain=300.0,
        pose_timeout_sec=0.5,
        rtde_control=None,
        rtde_receive=None,
        logger=None,
    ):
        if accepted_frame_ids is None:
            accepted_frame_ids = [expected_frame_id or "base"]

        if rclpy is None:
            raise RuntimeError("rclpy is required to run RTDEServoNode")
        super().__init__("rtde_servo_node")

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
        self._tcp_pose_frame_id = "base_link"
        self._tcp_pose_publisher = None

        self._param_node = self
        self._logger = logger or logging.getLogger("ur5_pose_tracker")
        self._logger = self.get_logger()
        self._declare_ros_parameters()
        self._load_ros_parameters()
        self.create_subscription(
            PoseStamped,
            self._input_topic,
            self._on_pose_msg,
            10,
        )
        self._tcp_pose_publisher = self.create_publisher(
            PoseStamped, "~/tcp_pose", 10
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
        if self._param_node is None:
            return
        self._param_node.declare_parameter("robot_ip", self._robot_ip)
        self._param_node.declare_parameter("input_topic", self._input_topic)
        self._param_node.declare_parameter("control_hz", self._control_hz)
        self._param_node.declare_parameter(
            "accepted_frame_ids", sorted(self._accepted_frame_ids)
        )
        self._param_node.declare_parameter("pose_timeout_sec", self._pose_timeout_sec)
        self._param_node.declare_parameter("servo_speed", self._speed)
        self._param_node.declare_parameter("servo_acceleration", self._acceleration)
        self._param_node.declare_parameter("servo_lookahead_time", self._lookahead_time)
        self._param_node.declare_parameter("servo_gain", int(self._gain))

    def _load_ros_parameters(self):
        if self._param_node is None:
            return
        self._robot_ip = str(self._param_node.get_parameter("robot_ip").value)
        self._input_topic = str(self._param_node.get_parameter("input_topic").value)
        self._control_hz = float(self._param_node.get_parameter("control_hz").value)
        frames = self._param_node.get_parameter("accepted_frame_ids").value
        self._accepted_frame_ids = {str(frame) for frame in frames if frame}
        if not self._accepted_frame_ids:
            self._accepted_frame_ids = {"base"}
        self._pose_timeout_sec = float(
            self._param_node.get_parameter("pose_timeout_sec").value
        )
        self._speed = float(self._param_node.get_parameter("servo_speed").value)
        self._acceleration = float(
            self._param_node.get_parameter("servo_acceleration").value
        )
        self._lookahead_time = float(
            self._param_node.get_parameter("servo_lookahead_time").value
        )
        self._gain = float(self._param_node.get_parameter("servo_gain").value)
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

    def _now_ros_time(self):
        return self.get_clock().now()

    def _read_actual_tcp_pose(self):
        tcp_pose = self._rtde_receive.getActualTCPPose()
        if tcp_pose is None or len(tcp_pose) != 6:
            raise ValueError("Invalid tcp pose data")
        try:
            return [float(value) for value in tcp_pose]
        except (TypeError, ValueError):
            raise ValueError("Invalid tcp pose data")

    def _build_tcp_pose_msg(self, tcp_pose):
        msg = PoseStamped()
        msg.header.stamp = self._now_ros_time().to_msg()
        msg.header.frame_id = self._tcp_pose_frame_id
        msg.pose.position.x = float(tcp_pose[0])
        msg.pose.position.y = float(tcp_pose[1])
        msg.pose.position.z = float(tcp_pose[2])
        qx, qy, qz, qw = self._rotvec_to_quat(
            float(tcp_pose[3]), float(tcp_pose[4]), float(tcp_pose[5])
        )
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        return msg

    def _publish_tcp_pose(self):
        if self._tcp_pose_publisher is None:
            return
        try:
            tcp_pose = self._read_actual_tcp_pose()
            msg = self._build_tcp_pose_msg(tcp_pose)
            self._tcp_pose_publisher.publish(msg)
        except Exception as exc:
            self._logger.warning(f"Failed to read tcp pose: {exc}")

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
                rclpy.spin_once(self, timeout_sec=0.0)
                if not rclpy.ok():
                    break

                self._publish_tcp_pose()
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

    @staticmethod
    def _rotvec_to_quat(rx, ry, rz):
        angle = math.sqrt(rx * rx + ry * ry + rz * rz)
        if angle < 1e-12:
            return 0.0, 0.0, 0.0, 1.0
        axis_x = rx / angle
        axis_y = ry / angle
        axis_z = rz / angle
        half = angle * 0.5
        sin_half = math.sin(half)
        return (
            axis_x * sin_half,
            axis_y * sin_half,
            axis_z * sin_half,
            math.cos(half),
        )


def main(args: Optional[list] = None):
    if rclpy is None:
        raise RuntimeError("rclpy is required to run RTDEServoNode")

    rclpy.init(args=args)
    node = RTDEServoNode()
    try:
        node.run_control_loop()
    except KeyboardInterrupt:
        node._logger.info("Shutdown requested.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
