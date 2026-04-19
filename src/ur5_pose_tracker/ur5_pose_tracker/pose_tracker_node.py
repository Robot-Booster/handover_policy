import math
import threading
import time
from typing import Any, List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface


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

        super().__init__("ur5_pose_tracker")

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
        self._running = True
        self._tcp_pose_frame_id = "base_link"
        self._tcp_pose_publisher = None
        self._base_frame = "base_link"
        self._ee_frame = "tool0"
        self._camera_frame = ""
        self._tf_broadcaster = None
        self._static_tf_broadcaster = None

        self._param_node = self
        # 日志 Logger: 单测注入 logger；运行时用 ROS logger（终端 / rosout）。
        self._logger = logger if logger is not None else self.get_logger()
        self._declare_ros_parameters()
        self._load_ros_parameters()
        startup_params_log = (
            "Node parameters: "
            f"robot_ip={self._robot_ip}, "
            f"input_topic={self._input_topic}, "
            f"control_hz={self._control_hz}, "
            f"accepted_frame_ids={sorted(self._accepted_frame_ids)}, "
            f"pose_timeout_sec={self._pose_timeout_sec}, "
            f"base_frame={self._base_frame}, ee_frame={self._ee_frame}, "
            f"camera_frame={self._camera_frame}, "
            f"servo_speed={self._speed}, "
            f"servo_acceleration={self._acceleration}, "
            f"servo_lookahead_time={self._lookahead_time}, "
            f"servo_gain={self._gain}"
        )
        self._logger.info(startup_params_log)
        self._logger.warning(
            "ur_rtde.servoL ignores servo_speed and servo_acceleration during motion "
            "(upstream API); tune control_hz, servo_lookahead_time, servo_gain. "
            "servo_acceleration is applied as tool deceleration [m/s^2] in servoStop()."
        )
        self._logger.info(f"Connecting robot RTDE interfaces: ip={self._robot_ip}")
        self.create_subscription(
            PoseStamped,
            self._input_topic,
            self._on_pose_msg,
            10,
        )
        self._tcp_pose_publisher = self.create_publisher(
            PoseStamped, "~/tcp_pose", 10
        )

        self._setup_tf()

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

        self._logger.info(f"Robot connected via RTDE: ip={self._robot_ip}")

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
        self._param_node.declare_parameter("base_frame", self._base_frame)
        self._param_node.declare_parameter("ee_frame", self._ee_frame)
        self._param_node.declare_parameter("camera_frame", self._camera_frame)
        self._param_node.declare_parameter("handeye_method", "")
        # 不能 declare(..., [])：rclpy 会用 from_parameter_value([]) 覆盖类型为 BYTE_ARRAY。
        self._param_node.declare_parameter(
            "handeye_tf.translation", Parameter.Type.DOUBLE_ARRAY
        )
        self._param_node.declare_parameter("handeye_tf.rotation", Parameter.Type.DOUBLE_ARRAY)

    def _load_ros_parameters(self):
        if self._param_node is None:
            return
        self._robot_ip = str(self._param_node.get_parameter("robot_ip").value)
        self._input_topic = str(self._param_node.get_parameter("input_topic").value)
        self._control_hz = float(self._param_node.get_parameter("control_hz").value)
        frames = self._param_node.get_parameter("accepted_frame_ids").value
        self._accepted_frame_ids = {str(frame) for frame in frames if frame}
        if not self._accepted_frame_ids:
            self._accepted_frame_ids = {"base_link"}
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
        self._base_frame = str(self._param_node.get_parameter("base_frame").value)
        self._ee_frame = str(self._param_node.get_parameter("ee_frame").value)
        self._camera_frame = str(self._param_node.get_parameter("camera_frame").value)
        self._tcp_pose_frame_id = self._base_frame

    def _setup_tf(self):
        """TF 广播：动态 base→ee；手眼静态在参数合法时发送一次。"""
        self._tf_broadcaster = TransformBroadcaster(self)
        self._static_tf_broadcaster = StaticTransformBroadcaster(self)
        method = ""
        if self._param_node is not None:
            method = str(self._param_node.get_parameter("handeye_method").value)
        tvec, rmat, parent, child = self._parse_handeye_static(method)
        if tvec is None:
            self._logger.info(
                "TF: no static hand-eye on /tf_static (see WARN above if any). "
                "Dynamic /tf (base->ee) only after RTDE TCP pose reads succeed."
            )
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
        # tf2_ros API: list form；/tf_static 为 TRANSIENT_LOCAL，晚订阅的节点仍可收到。
        self._static_tf_broadcaster.sendTransform([st])
        self._logger.info(
            f"TF: published static transform {parent} -> {child} on /tf_static"
        )

    def _parse_handeye_static(
        self, method: str
    ) -> Tuple[
        Optional[Sequence[float]],
        Optional[List[List[float]]],
        Optional[str],
        Optional[str],
    ]:
        """解析手眼静态 TF；失败则 WARN 并返回全 None。"""
        if self._param_node is None:
            self._logger.warning("Hand-eye static TF skipped: no parameter node.")
            return None, None, None, None
        raw_t = self._param_node.get_parameter("handeye_tf.translation").value
        raw_r = self._param_node.get_parameter("handeye_tf.rotation").value
        tvec = self._coerce_translation(raw_t)
        rmat = self._coerce_rotation_matrix(raw_r)
        if tvec is None or rmat is None:
            self._logger.warning(
                "Hand-eye static TF skipped: invalid or missing handeye_tf.translation / "
                "handeye_tf.rotation (translation: length 3; rotation: 9 floats row-major)."
            )
            return None, None, None, None
        m = method.strip()
        if not m:
            self._logger.warning("Hand-eye static TF skipped: handeye_method not set.")
            return None, None, None, None
        if m == "eye_on_hand":
            if not str(self._ee_frame).strip():
                self._logger.warning(
                    "Hand-eye static TF skipped: ee_frame is empty for eye_on_hand."
                )
                return None, None, None, None
            if not str(self._camera_frame).strip():
                self._logger.warning(
                    "Hand-eye static TF skipped: camera_frame is empty for eye_on_hand."
                )
                return None, None, None, None
            return tvec, rmat, self._ee_frame, self._camera_frame
        if m == "eye_on_base":
            if not str(self._base_frame).strip():
                self._logger.warning(
                    "Hand-eye static TF skipped: base_frame is empty for eye_on_base."
                )
                return None, None, None, None
            if not str(self._camera_frame).strip():
                self._logger.warning(
                    "Hand-eye static TF skipped: camera_frame is empty for eye_on_base."
                )
                return None, None, None, None
            return tvec, rmat, self._base_frame, self._camera_frame
        self._logger.warning(
            f"Hand-eye static TF skipped: unknown handeye_method={method!r} "
            '(expected "eye_on_hand" or "eye_on_base").'
        )
        return None, None, None, None

    def _coerce_translation(self, raw: Any) -> Optional[Tuple[float, float, float]]:
        if raw is None:
            return None
        if not isinstance(raw, (list, tuple)) or len(raw) != 3:
            return None
        try:
            return (float(raw[0]), float(raw[1]), float(raw[2]))
        except (TypeError, ValueError):
            return None

    def _coerce_rotation_matrix(self, raw: Any) -> Optional[List[List[float]]]:
        if raw is None:
            return None
        if not isinstance(raw, (list, tuple)) or len(raw) != 9:
            return None
        try:
            v = [float(raw[i]) for i in range(9)]
        except (TypeError, ValueError, IndexError):
            return None
        return [
            [v[0], v[1], v[2]],
            [v[3], v[4], v[5]],
            [v[6], v[7], v[8]],
        ]

    @staticmethod
    def _rotmat_to_quat_xyzw(r: List[List[float]]) -> Tuple[float, float, float, float]:
        """旋转矩阵 → 四元数 xyzw（行主序）。"""
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
        qx, qy, qz, qw = self._rotvec_to_quat(
            float(tcp_pose[3]), float(tcp_pose[4]), float(tcp_pose[5])
        )
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
        # ur_rtde servoL：speed/acceleration 形参库内未用于轨迹，运动由 dt、lookahead、gain 决定。
        self._rtde_control.servoL(
            target,
            self._speed,
            self._acceleration,
            self._dt,
            self._lookahead_time,
            self._gain,
        )

    def _servo_stop_tool_deceleration_mss(self):
        """servoStop 工具减速率 [m/s^2]，与 YAML servo_acceleration 一致。"""
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
            self._publish_dynamic_base_to_ee(tcp_pose)
        except Exception as exc:
            self._logger.warning(f"Failed to read tcp pose: {exc}")

    def _safe_stop(self):
        try:
            self._rtde_control.servoStop(self._servo_stop_tool_deceleration_mss())
        except Exception as exc:  # pragma: no cover - hardware safety path
            self._logger.error(f"servoStop failed: {exc}")

    def _disconnect_rtde(self):
        control = getattr(self, "_rtde_control", None)
        receive = getattr(self, "_rtde_receive", None)
        try:
            if control is not None:
                if hasattr(control, "stopScript"):
                    control.stopScript()
                if hasattr(control, "disconnect"):
                    control.disconnect()
            if receive is not None and hasattr(receive, "disconnect"):
                receive.disconnect()
            self._logger.info(f"Robot disconnected: ip={self._robot_ip}")
        except Exception as exc:  # pragma: no cover - hardware cleanup path
            self._logger.warning(f"RTDE disconnect warning: {exc}")

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
