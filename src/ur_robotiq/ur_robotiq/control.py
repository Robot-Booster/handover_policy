import math
import threading
import time
from typing import Any, List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

try:
    from pyrobotiqgripper import RobotiqGripper
except Exception:  # pragma: no cover - optional runtime dependency
    RobotiqGripper = None

try:
    from ur_robotiq_interfaces.srv import MoveToPose, SetGripper
except Exception:  # pragma: no cover - tests without generated interfaces
    MoveToPose = None
    SetGripper = None


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
            accepted_frame_ids = [expected_frame_id or "base_link"]

        super().__init__("ur_robotiq")
        self._robot_ip = str(robot_ip)
        self._input_topic = str(input_topic)
        self._accepted_frame_ids = {str(frame) for frame in accepted_frame_ids if frame}
        if not self._accepted_frame_ids:
            self._accepted_frame_ids = {"base_link"}
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
        self._mode_lock = threading.Lock()
        self._running = True
        self._control_mode = "IDLE"
        self._target_seq = 0
        self._movel_seq_gate = 0
        self._tcp_pose_frame_id = "base_link"
        self._tcp_pose_publisher = None
        self._base_frame = "base_link"
        self._ee_frame = "tool0"
        self._camera_frame = ""
        self._tf_broadcaster = None
        self._static_tf_broadcaster = None
        self._gripper = None
        self._gripper_enabled = False
        self._gripper_com_port = ""
        self._gripper_speed = 255
        self._gripper_force = 255

        self._param_node = self
        self._logger = logger if logger is not None else self.get_logger()
        self._declare_ros_parameters()
        self._load_ros_parameters()
        self._logger.info(
            "Node parameters: "
            f"robot_ip={self._robot_ip}, input_topic={self._input_topic}, "
            f"control_hz={self._control_hz}, accepted_frame_ids={sorted(self._accepted_frame_ids)}, "
            f"pose_timeout_sec={self._pose_timeout_sec}, base_frame={self._base_frame}, "
            f"ee_frame={self._ee_frame}, camera_frame={self._camera_frame}, "
            f"speed={self._speed}, acceleration={self._acceleration}, "
            f"servo_lookahead_time={self._lookahead_time}, servo_gain={self._gain}"
        )
        self._logger.info(f"Connecting robot RTDE interfaces: ip={self._robot_ip}")
        self.create_subscription(PoseStamped, self._input_topic, self._on_pose_msg, 1)
        self._tcp_pose_publisher = self.create_publisher(PoseStamped, "~/tcp_pose", 10)
        self._setup_tf()
        self._create_services()

        self._rtde_control = rtde_control or RTDEControlInterface(self._robot_ip)
        self._rtde_receive = rtde_receive or RTDEReceiveInterface(self._robot_ip)
        self._logger.info(f"Robot connected via RTDE: ip={self._robot_ip}")
        self._init_gripper()

    def _declare_ros_parameters(self):
        if self._param_node is None:
            return
        self._param_node.declare_parameter("robot_ip", self._robot_ip)
        self._param_node.declare_parameter("input_topic", self._input_topic)
        self._param_node.declare_parameter("control_hz", self._control_hz)
        self._param_node.declare_parameter("accepted_frame_ids", sorted(self._accepted_frame_ids))
        self._param_node.declare_parameter("pose_timeout_sec", self._pose_timeout_sec)
        self._param_node.declare_parameter("speed", self._speed)
        self._param_node.declare_parameter("acceleration", self._acceleration)
        self._param_node.declare_parameter("servo_lookahead_time", self._lookahead_time)
        self._param_node.declare_parameter("servo_gain", int(self._gain))
        self._param_node.declare_parameter("base_frame", self._base_frame)
        self._param_node.declare_parameter("ee_frame", self._ee_frame)
        self._param_node.declare_parameter("camera_frame", self._camera_frame)
        self._param_node.declare_parameter("gripper_com_port", self._gripper_com_port)
        self._param_node.declare_parameter("gripper_speed", int(self._gripper_speed))
        self._param_node.declare_parameter("gripper_force", int(self._gripper_force))
        self._param_node.declare_parameter("handeye_method", "")
        self._param_node.declare_parameter("handeye_tf.translation", Parameter.Type.DOUBLE_ARRAY)
        self._param_node.declare_parameter("handeye_tf.rotation", Parameter.Type.DOUBLE_ARRAY)

    def _load_ros_parameters(self):
        self._robot_ip = str(self._param_node.get_parameter("robot_ip").value)
        self._input_topic = str(self._param_node.get_parameter("input_topic").value)
        self._control_hz = float(self._param_node.get_parameter("control_hz").value)
        self._accepted_frame_ids = {
            str(frame) for frame in self._param_node.get_parameter("accepted_frame_ids").value if frame
        } or {"base_link"}
        self._pose_timeout_sec = float(self._param_node.get_parameter("pose_timeout_sec").value)
        self._speed = float(self._param_node.get_parameter("speed").value)
        self._acceleration = float(self._param_node.get_parameter("acceleration").value)
        self._lookahead_time = float(self._param_node.get_parameter("servo_lookahead_time").value)
        self._gain = float(self._param_node.get_parameter("servo_gain").value)
        self._dt = 1.0 / self._control_hz
        self._base_frame = str(self._param_node.get_parameter("base_frame").value)
        self._ee_frame = str(self._param_node.get_parameter("ee_frame").value)
        self._camera_frame = str(self._param_node.get_parameter("camera_frame").value)
        self._gripper_com_port = str(self._param_node.get_parameter("gripper_com_port").value)
        self._gripper_speed = int(self._param_node.get_parameter("gripper_speed").value)
        self._gripper_force = int(self._param_node.get_parameter("gripper_force").value)
        self._tcp_pose_frame_id = self._base_frame

    def _create_services(self):
        if MoveToPose is not None:
            self.create_service(MoveToPose, "~/move_l", self._handle_move_l)
        if SetGripper is not None:
            self.create_service(SetGripper, "~/set_gripper", self._handle_set_gripper)

    def _init_gripper(self):
        if not self._gripper_com_port:
            self._logger.error("gripper_com_port is required, gripper control disabled.")
            return
        if RobotiqGripper is None:
            self._logger.error("pyrobotiqgripper is unavailable, gripper control disabled.")
            return
        try:
            self._gripper = RobotiqGripper(com_port=self._gripper_com_port)
            self._gripper.activate()
            self._gripper_enabled = True
            self._logger.info(f"Gripper connected: com_port={self._gripper_com_port}")
        except Exception as exc:
            self._logger.error(f"Failed to init gripper: {exc}")

    def _setup_tf(self):
        self._tf_broadcaster = TransformBroadcaster(self)
        self._static_tf_broadcaster = StaticTransformBroadcaster(self)
        method = str(self._param_node.get_parameter("handeye_method").value)
        tvec, rmat, parent, child = self._parse_handeye_static(method)
        if tvec is None:
            self._logger.info("TF: no static hand-eye on /tf_static (see WARN above if any).")
            return
        st = TransformStamped()
        st.header.stamp = self._now_ros_time().to_msg()
        st.header.frame_id = parent
        st.child_frame_id = child
        st.transform.translation.x = float(tvec[0])
        st.transform.translation.y = float(tvec[1])
        st.transform.translation.z = float(tvec[2])
        qx, qy, qz, qw = self._rotmat_to_quat_xyzw(rmat)
        st.transform.rotation.x = qx
        st.transform.rotation.y = qy
        st.transform.rotation.z = qz
        st.transform.rotation.w = qw
        self._static_tf_broadcaster.sendTransform([st])
        self._logger.info(f"TF: published static transform {parent} -> {child} on /tf_static")

    def _parse_handeye_static(self, method: str):
        try:
            raw_t = self._param_node.get_parameter("handeye_tf.translation").value
        except Exception:
            raw_t = None
        try:
            raw_r = self._param_node.get_parameter("handeye_tf.rotation").value
        except Exception:
            raw_r = None
        tvec = self._coerce_translation(raw_t)
        rmat = self._coerce_rotation_matrix(raw_r)
        if tvec is None or rmat is None:
            self._logger.warning("Hand-eye static TF skipped: invalid or missing handeye_tf.")
            return None, None, None, None
        if method == "eye_on_hand":
            return tvec, rmat, self._ee_frame, self._camera_frame
        if method == "eye_on_base":
            return tvec, rmat, self._base_frame, self._camera_frame
        self._logger.warning("Hand-eye static TF skipped: handeye_method not set or invalid.")
        return None, None, None, None

    def _coerce_translation(self, raw: Any):
        if not isinstance(raw, (list, tuple)) or len(raw) != 3:
            return None
        try:
            return (float(raw[0]), float(raw[1]), float(raw[2]))
        except (TypeError, ValueError):
            return None

    def _coerce_rotation_matrix(self, raw: Any):
        if not isinstance(raw, (list, tuple)) or len(raw) != 9:
            return None
        try:
            v = [float(raw[i]) for i in range(9)]
        except (TypeError, ValueError, IndexError):
            return None
        return [[v[0], v[1], v[2]], [v[3], v[4], v[5]], [v[6], v[7], v[8]]]

    @staticmethod
    def _rotmat_to_quat_xyzw(r: List[List[float]]):
        m00, m01, m02 = r[0]
        m10, m11, m12 = r[1]
        m20, m21, m22 = r[2]
        tr = m00 + m11 + m22
        if tr > 0.0:
            s = 0.5 / math.sqrt(tr + 1.0)
            qw = 0.25 / s
            qx = (m21 - m12) * s
            qy = (m02 - m20) * s
            qz = (m10 - m01) * s
        elif m00 > m11 and m00 > m22:
            s = 2.0 * math.sqrt(1.0 + m00 - m11 - m22)
            qw = (m21 - m12) / s
            qx = 0.25 * s
            qy = (m01 + m10) / s
            qz = (m02 + m20) / s
        elif m11 > m22:
            s = 2.0 * math.sqrt(1.0 + m11 - m00 - m22)
            qw = (m02 - m20) / s
            qx = (m01 + m10) / s
            qy = 0.25 * s
            qz = (m12 + m21) / s
        else:
            s = 2.0 * math.sqrt(1.0 + m22 - m00 - m11)
            qw = (m10 - m01) / s
            qx = (m02 + m20) / s
            qy = (m12 + m21) / s
            qz = 0.25 * s
        n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if n <= 0.0:
            return 0.0, 0.0, 0.0, 1.0
        return qx / n, qy / n, qz / n, qw / n

    def _publish_dynamic_base_to_ee(self, tcp_pose):
        if self._tf_broadcaster is None:
            return
        t = TransformStamped()
        t.header.stamp = self._now_ros_time().to_msg()
        t.header.frame_id = self._base_frame
        t.child_frame_id = self._ee_frame
        t.transform.translation.x = float(tcp_pose[0])
        t.transform.translation.y = float(tcp_pose[1])
        t.transform.translation.z = float(tcp_pose[2])
        qx, qy, qz, qw = self._rotvec_to_quat(float(tcp_pose[3]), float(tcp_pose[4]), float(tcp_pose[5]))
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform([t])

    def _on_pose_msg(self, msg):
        if not self._validate_frame(msg.header.frame_id):
            self._logger.warning(f"Ignore pose with invalid frame: {msg.header.frame_id}")
            return
        target = self._pose_to_servol_target(msg.pose)
        with self._target_lock:
            self._latest_target = target
            self._latest_target_time = time.monotonic()
            self._target_seq += 1

    def _validate_frame(self, frame_id):
        return bool(frame_id) and frame_id in self._accepted_frame_ids

    def _pose_to_servol_target(self, pose):
        qx = float(pose.orientation.x)
        qy = float(pose.orientation.y)
        qz = float(pose.orientation.z)
        qw = float(pose.orientation.w)
        rx, ry, rz = self._quat_to_rotvec(qx, qy, qz, qw)
        return [float(pose.position.x), float(pose.position.y), float(pose.position.z), rx, ry, rz]

    def _send_servo(self, target):
        self._rtde_control.servoL(target, self._speed, self._acceleration, self._dt, self._lookahead_time, self._gain)
        with self._mode_lock:
            if self._control_mode in ("IDLE", "SERVO"):
                self._control_mode = "SERVO"

    def _execute_movel(self, target):
        self._rtde_control.moveL(target, self._speed, self._acceleration)

    def _handle_move_l(self, request, response):
        if not self._validate_frame(request.target.header.frame_id):
            response.success = False
            response.message = "invalid frame_id"
            return response
        target = self._pose_to_servol_target(request.target.pose)
        self._logger.info(f"moveL request received: frame={request.target.header.frame_id}")
        with self._mode_lock:
            with self._target_lock:
                self._movel_seq_gate = self._target_seq
            if self._control_mode == "SERVO":
                self._safe_stop()
                self._logger.info("servo stopped, switching to moveL")
            self._control_mode = "MOVEL"
        try:
            self._execute_movel(target)
            response.success = True
            response.message = "moveL reached target"
            self._logger.info("moveL reached target")
        except Exception as exc:
            response.success = False
            response.message = f"moveL failed: {exc}"
            self._logger.error(response.message)
        finally:
            with self._mode_lock:
                self._control_mode = "IDLE"
            self._logger.info("waiting for new servo target")
        return response

    def _handle_set_gripper(self, request, response):
        if not self._gripper_enabled:
            response.success = False
            response.message = "gripper is disabled"
            return response
        position = float(request.position)
        if position < 0.0 or position > 1.0:
            response.success = False
            response.message = "position must be in [0, 1]"
            return response
        target_bit = int(round(position * 255.0))
        self._logger.info(f"gripper target received: {position:.3f}")
        try:
            self._gripper.move(target_bit, speed=self._gripper_speed, force=self._gripper_force, wait=True)
            self._logger.info(f"gripper move done: bit={target_bit}")
            response.success = True
            response.message = "ok"
        except Exception as exc:
            response.success = False
            response.message = f"gripper move failed: {exc}"
            self._logger.error(response.message)
        return response

    def _servo_stop_tool_deceleration_mss(self):
        a = float(self._acceleration)
        if a <= 0.0:
            return 10.0
        return a

    def _now_ros_time(self):
        return self.get_clock().now()

    def _read_actual_tcp_pose(self):
        tcp_pose = self._rtde_receive.getActualTCPPose()
        if tcp_pose is None or len(tcp_pose) != 6:
            raise ValueError("Invalid tcp pose data")
        return [float(value) for value in tcp_pose]

    def _build_tcp_pose_msg(self, tcp_pose):
        msg = PoseStamped()
        msg.header.stamp = self._now_ros_time().to_msg()
        msg.header.frame_id = self._tcp_pose_frame_id
        msg.pose.position.x = float(tcp_pose[0])
        msg.pose.position.y = float(tcp_pose[1])
        msg.pose.position.z = float(tcp_pose[2])
        qx, qy, qz, qw = self._rotvec_to_quat(float(tcp_pose[3]), float(tcp_pose[4]), float(tcp_pose[5]))
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
            self._publish_dynamic_base_to_ee(tcp_pose)
        except Exception as exc:
            self._logger.warning(f"Failed to read tcp pose: {exc}")

    def _safe_stop(self):
        try:
            self._rtde_control.servoStop(self._servo_stop_tool_deceleration_mss())
        except Exception as exc:
            self._logger.error(f"servoStop failed: {exc}")

    def _disconnect_rtde(self):
        try:
            if hasattr(self._rtde_control, "stopScript"):
                self._rtde_control.stopScript()
            if hasattr(self._rtde_control, "disconnect"):
                self._rtde_control.disconnect()
            if hasattr(self._rtde_receive, "disconnect"):
                self._rtde_receive.disconnect()
            self._logger.info(f"Robot disconnected: ip={self._robot_ip}")
        except Exception as exc:
            self._logger.warning(f"RTDE disconnect warning: {exc}")

    def run_control_loop(self, max_steps=None):
        step = 0
        try:
            while self._running:
                with self._target_lock:
                    target = self._latest_target
                    target_time = self._latest_target_time
                    fresh_after_movel = self._target_seq > self._movel_seq_gate
                with self._mode_lock:
                    mode = self._control_mode
                if target is not None and mode != "MOVEL":
                    if mode == "IDLE" and not fresh_after_movel:
                        pass
                    elif (time.monotonic() - target_time) > self._pose_timeout_sec:
                        self._logger.warning("Pose timeout, stopping servo.")
                        self._safe_stop()
                        with self._mode_lock:
                            self._control_mode = "IDLE"
                        with self._target_lock:
                            self._latest_target = None
                            self._latest_target_time = 0.0
                    else:
                        try:
                            self._send_servo(target)
                        except Exception as exc:
                            self._logger.error(f"servoL failed: {exc}")
                            self._safe_stop()
                            with self._mode_lock:
                                self._control_mode = "IDLE"
                            with self._target_lock:
                                self._latest_target = None
                                self._latest_target_time = 0.0
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
            self._disconnect_rtde()

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
        return qx / sin_half * angle, qy / sin_half * angle, qz / sin_half * angle

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
        return (axis_x * sin_half, axis_y * sin_half, axis_z * sin_half, math.cos(half))


def main(args: Optional[list] = None):
    rclpy.init(args=args)
    node = RTDEServoNode()
    try:
        node.run_control_loop()
    except KeyboardInterrupt:
        node._logger.info("Shutdown requested.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
