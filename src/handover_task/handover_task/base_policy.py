import json
import math
import time

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from visualization_msgs.msg import Marker

from handover_task.linear_cv_kalman import LinearCvKalmanFilter

try:
    from handover_task_interfaces.srv import BasePolicy
except Exception:  # pragma: no cover
    BasePolicy = None

try:
    from ur_robotiq_interfaces.srv import MoveToPose, SetGripper
except Exception:  # pragma: no cover
    MoveToPose = None
    SetGripper = None


class _WorkspaceObb:
    def __init__(self, center, half_extents, yaw_deg, pitch_deg, roll_deg):
        self.center = [float(v) for v in center]
        self.half_extents = [float(v) for v in half_extents]
        self.yaw_deg = float(yaw_deg)
        self.pitch_deg = float(pitch_deg)
        self.roll_deg = float(roll_deg)
        self._rot = self._rotation_matrix_zyx(self.yaw_deg, self.pitch_deg, self.roll_deg)
        self._rot_t = self._transpose3(self._rot)

    @classmethod
    def from_json(cls, json_path):
        with open(json_path, "r", encoding="utf-8") as f:
            data = json.load(f)
        if data.get("euler_deg_order", "ZYX") != "ZYX":
            raise ValueError("workspace euler_deg_order must be ZYX")
        return cls(
            center=data["center"],
            half_extents=data["half_extents"],
            yaw_deg=data["yaw_deg"],
            pitch_deg=data["pitch_deg"],
            roll_deg=data["roll_deg"],
        )

    def clamp(self, p_xyz):
        delta = [float(p_xyz[i]) - self.center[i] for i in range(3)]
        local = self._mul3(self._rot_t, delta)
        clamped_local = [
            min(max(local[i], -self.half_extents[i]), self.half_extents[i]) for i in range(3)
        ]
        world = self._mul3(self._rot, clamped_local)
        return [self.center[i] + world[i] for i in range(3)]

    @staticmethod
    def _transpose3(m):
        return [[m[0][0], m[1][0], m[2][0]], [m[0][1], m[1][1], m[2][1]], [m[0][2], m[1][2], m[2][2]]]

    @staticmethod
    def _mul3(m, v):
        return [
            m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
            m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
            m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
        ]

    @staticmethod
    def _rotation_matrix_zyx(yaw_deg, pitch_deg, roll_deg):
        yaw = math.radians(yaw_deg)
        pitch = math.radians(pitch_deg)
        roll = math.radians(roll_deg)
        cy, sy = math.cos(yaw), math.sin(yaw)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cr, sr = math.cos(roll), math.sin(roll)
        return [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]


class BasePolicyNode(Node):
    def __init__(self):
        super().__init__("base_policy")
        self._logger = self.get_logger()
        self._state = "IDLE"

        input_point_topic = str(self.declare_parameter("input_point_topic", "~/target_point").value)
        input_tcp_pose_topic = str(self.declare_parameter("input_tcp_pose_topic", "~/tcp_pose").value)
        output_grasp_pose_topic = str(self.declare_parameter("output_grasp_pose_topic", "~/grasp_pose").value)
        output_estimated_point_topic = str(
            self.declare_parameter("output_estimated_point_topic", "~/estimated_target_point").value
        )
        self._workspace_marker_topic = str(
            self.declare_parameter("workspace_marker_topic", "~/workspace_marker").value
        )
        self._output_frame_id = str(self.declare_parameter("output_frame_id", "base_link").value)
        update_hz = float(self.declare_parameter("update_hz", 20.0).value)
        self._feedforward_sec = float(self.declare_parameter("feedforward_sec", 0.2).value)
        self._lost_timeout_sec = float(self.declare_parameter("lost_timeout_sec", 0.2).value)
        self._workspace_box_json_path = str(self.declare_parameter("workspace_box_json_path", "").value).strip()
        self._workspace_marker_rgba = [
            float(v) for v in self.declare_parameter("workspace_marker_rgba", [0.0, 0.0, 1.0, 0.25]).value
        ]

        self._tcp_z_offset_m = float(self.declare_parameter("tcp_z_offset_m", -0.2).value)
        self._tcp_z_approach_m = float(self.declare_parameter("tcp_z_approach_m", 0.05).value)
        self._task_timeout_sec = float(self.declare_parameter("task_timeout_sec", 10.0).value)
        self._grasp_distance_tolerance_m = float(self.declare_parameter("grasp_distance_tolerance_m", 0.015).value)
        self._grasp_stable_duration_sec = float(self.declare_parameter("grasp_stable_duration_sec", 0.4).value)
        self._use_custom_return_pose = bool(self.declare_parameter("use_custom_return_pose", False).value)
        self._custom_return_position = [float(v) for v in self.declare_parameter("custom_return_pose.position", [0.0, 0.0, 0.0]).value]
        self._custom_return_orientation = [
            float(v) for v in self.declare_parameter("custom_return_pose.orientation_xyzw", [0.0, 0.0, 0.0, 1.0]).value
        ]
        self._close_gripper_position = float(self.declare_parameter("close_gripper_position", 1.0).value)
        self._open_gripper_position = float(self.declare_parameter("open_gripper_position", 0.0).value)
        self._move_l_service_name = str(self.declare_parameter("move_l_service", "/ur_robotiq/move_l").value)
        self._set_gripper_service_name = str(
            self.declare_parameter("set_gripper_service", "/ur_robotiq/set_gripper").value
        )
        if not self._workspace_box_json_path:
            raise RuntimeError("workspace_box_json_path is empty")
        self._workspace = _WorkspaceObb.from_json(self._workspace_box_json_path)
        process_noise_pos = float(self.declare_parameter("process_noise_pos", 1.0e-4).value)
        process_noise_vel = float(self.declare_parameter("process_noise_vel", 1.0e-2).value)
        measure_noise_pos = float(self.declare_parameter("measure_noise_pos", 1.0e-3).value)
        self._kf = LinearCvKalmanFilter(
            process_noise_pos=process_noise_pos,
            process_noise_vel=process_noise_vel,
            measure_noise_pos=measure_noise_pos,
        )
        self._logger.info(
            "Node parameters:\n"
            f"  input_point_topic={input_point_topic}\n"
            f"  input_tcp_pose_topic={input_tcp_pose_topic}\n"
            f"  output_grasp_pose_topic={output_grasp_pose_topic}\n"
            f"  output_estimated_point_topic={output_estimated_point_topic}\n"
            f"  workspace_marker_topic={self._workspace_marker_topic}\n"
            f"  output_frame_id={self._output_frame_id}\n"
            f"  update_hz={update_hz}\n"
            f"  process_noise_pos={process_noise_pos}\n"
            f"  process_noise_vel={process_noise_vel}\n"
            f"  measure_noise_pos={measure_noise_pos}\n"
            f"  feedforward_sec={self._feedforward_sec}\n"
            f"  lost_timeout_sec={self._lost_timeout_sec}\n"
            f"  workspace_box_json_path={self._workspace_box_json_path}\n"
            f"  workspace_marker_rgba={self._workspace_marker_rgba}\n"
            f"  tcp_z_offset_m={self._tcp_z_offset_m}\n"
            f"  tcp_z_approach_m={self._tcp_z_approach_m}\n"
            f"  task_timeout_sec={self._task_timeout_sec}\n"
            f"  grasp_distance_tolerance_m={self._grasp_distance_tolerance_m}\n"
            f"  grasp_stable_duration_sec={self._grasp_stable_duration_sec}\n"
            f"  use_custom_return_pose={self._use_custom_return_pose}\n"
            f"  custom_return_pose.position={self._custom_return_position}\n"
            f"  custom_return_pose.orientation_xyzw={self._custom_return_orientation}\n"
            f"  close_gripper_position={self._close_gripper_position}\n"
            f"  open_gripper_position={self._open_gripper_position}\n"
            f"  move_l_service={self._move_l_service_name}\n"
            f"  set_gripper_service={self._set_gripper_service_name}"
        )

        self._grasp_pose_pub = self.create_publisher(PoseStamped, output_grasp_pose_topic, 10)
        self._estimated_point_pub = self.create_publisher(PointStamped, output_estimated_point_topic, 10)
        self._workspace_marker_pub = self.create_publisher(Marker, self._workspace_marker_topic, 10)

        self._last_target_xyz = None
        self._last_target_wall_time = None
        self._last_tick_wall_time = None
        self._latest_grasp_pose = None
        self._latest_object_pose = None
        self._latest_tcp_pose = None
        self._initial_tcp_pose = None
        self._kf_initialized = False
        self._control_enabled = False

        self._tracking_cb_group = ReentrantCallbackGroup()
        self._service_cb_group = MutuallyExclusiveCallbackGroup()
        self.create_subscription(
            PointStamped, input_point_topic, self._on_target_point, 10, callback_group=self._tracking_cb_group
        )
        self.create_subscription(
            PoseStamped, input_tcp_pose_topic, self._on_tcp_pose, 10, callback_group=self._tracking_cb_group
        )
        self.create_timer(1.0 / max(update_hz, 1.0), self._on_timer, callback_group=self._tracking_cb_group)

        self._move_l_client = self.create_client(MoveToPose, self._move_l_service_name) if MoveToPose else None
        self._set_gripper_client = self.create_client(SetGripper, self._set_gripper_service_name) if SetGripper else None
        self._srv_running = False
        if BasePolicy is not None:
            self.create_service(
                BasePolicy, "~/base_policy", self._on_base_policy_service, callback_group=self._service_cb_group
            )

        self._logger.info("base_policy initialized")

    def run_once_handover(self):
        if self._srv_running:
            return False, "FAIL_BUSY", "task is already running"
        self._srv_running = True
        self._control_enabled = True
        try:
            if self._initial_tcp_pose is None:
                return False, "FAIL_RETURN_INITIAL", "initial tcp pose is unavailable"
            ok, code, msg = self._track_until_stable()
            if not ok:
                return False, code, msg
            # 追踪稳定后停止下发 servo 目标 / stop servo target publishing after stable tracking.
            self._control_enabled = False
            ok, code, msg = self._execute_grasp()
            if not ok:
                return False, code, msg
            ok, code, msg = self._go_handover_pose()
            if not ok:
                return False, code, msg
            ok, code, msg = self._release_gripper()
            if not ok:
                return False, code, msg
            ok, code, msg = self._go_initial_pose()
            if not ok:
                return False, code, msg
            return True, "OK", "task complete"
        finally:
            self._control_enabled = False
            self._srv_running = False

    def _on_base_policy_service(self, request, response):
        _ = request
        success, code, message = self.run_once_handover()
        response.success = bool(success)
        response.code = str(code)
        response.message = str(message)
        if success:
            self._logger.info(f"base_policy finished: code={code}")
        else:
            self._logger.warning(f"base_policy failed: code={code}, message={message}")
        return response

    def _track_until_stable(self):
        start = time.monotonic()
        stable_begin = None
        while True:
            now = time.monotonic()
            if now - start > self._task_timeout_sec:
                return False, "FAIL_TIMEOUT", "task timeout"
            if self._latest_object_pose is None or self._latest_tcp_pose is None:
                time.sleep(0.01)
                continue
            # 用 TCP 反向抵消追踪偏移后，再和物体估计点比较距离 / compare against object estimate after tcp offset compensation.
            aligned_tcp_pose = self._build_tcp_forward_pose(self._latest_tcp_pose, -self._tcp_z_offset_m)
            distance = self._distance_between_pose(self._latest_object_pose, aligned_tcp_pose)
            if distance <= self._grasp_distance_tolerance_m:
                if stable_begin is None:
                    stable_begin = now
                elif now - stable_begin >= self._grasp_stable_duration_sec:
                    return True, "OK", "tracking stable"
            else:
                stable_begin = None
            time.sleep(0.01)

    def _execute_grasp(self):
        if self._latest_tcp_pose is None:
            return False, "FAIL_GRASP_MOVE", "tcp pose is unavailable"
        approach = self._build_tcp_forward_pose(self._latest_tcp_pose, self._tcp_z_approach_m)
        ok, msg = self._call_move_l(approach)
        if not ok:
            return False, "FAIL_GRASP_MOVE", msg
        ok, msg = self._call_set_gripper(self._close_gripper_position)
        if not ok:
            return False, "FAIL_GRIPPER", msg
        return True, "OK", "grasp complete"

    def _go_handover_pose(self):
        if self._use_custom_return_pose:
            try:
                target = self._build_custom_return_pose()
            except Exception as exc:
                return False, "FAIL_HANDOVER_POSE", f"invalid custom return pose: {exc}"
            ok, msg = self._call_move_l(target)
            if not ok:
                return False, "FAIL_HANDOVER_POSE", msg
            return True, "OK", "moved to custom handover pose"
        if self._initial_tcp_pose is None:
            return False, "FAIL_HANDOVER_POSE", "initial tcp pose is unavailable"
        ok, msg = self._call_move_l(self._initial_tcp_pose)
        if not ok:
            return False, "FAIL_HANDOVER_POSE", msg
        return True, "OK", "handover pose uses initial pose"

    def _release_gripper(self):
        ok, msg = self._call_set_gripper(self._open_gripper_position)
        if not ok:
            return False, "FAIL_GRIPPER", msg
        return True, "OK", "gripper opened at handover pose"

    def _go_initial_pose(self):
        if self._initial_tcp_pose is None:
            return False, "FAIL_RETURN_INITIAL", "initial tcp pose is unavailable"
        ok, msg = self._call_move_l(self._initial_tcp_pose)
        if not ok:
            return False, "FAIL_RETURN_INITIAL", msg
        return True, "OK", "returned to initial pose"

    def _call_move_l(self, target_pose):
        if self._move_l_client is None:
            return False, "move_l client is unavailable"
        if not self._move_l_client.wait_for_service(timeout_sec=1.0):
            return False, "move_l service unavailable"
        req = MoveToPose.Request()
        req.target = target_pose
        future = self._move_l_client.call_async(req)
        while rclpy.ok() and not future.done():
            time.sleep(0.01)
        if not future.done():
            return False, "move_l call interrupted"
        result = future.result()
        if result is None:
            return False, "move_l returned no response"
        return bool(result.success), str(result.message)

    def _call_set_gripper(self, position):
        if self._set_gripper_client is None:
            return False, "set_gripper client is unavailable"
        if not self._set_gripper_client.wait_for_service(timeout_sec=1.0):
            return False, "set_gripper service unavailable"
        req = SetGripper.Request()
        req.position = float(position)
        future = self._set_gripper_client.call_async(req)
        while rclpy.ok() and not future.done():
            time.sleep(0.01)
        if not future.done():
            return False, "set_gripper call interrupted"
        result = future.result()
        if result is None:
            return False, "set_gripper returned no response"
        return bool(result.success), str(result.message)

    def _build_custom_return_pose(self):
        if len(self._custom_return_position) != 3 or len(self._custom_return_orientation) != 4:
            raise ValueError("custom_return_pose shape mismatch")
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self._output_frame_id
        pose.pose.position.x = float(self._custom_return_position[0])
        pose.pose.position.y = float(self._custom_return_position[1])
        pose.pose.position.z = float(self._custom_return_position[2])
        pose.pose.orientation.x = float(self._custom_return_orientation[0])
        pose.pose.orientation.y = float(self._custom_return_orientation[1])
        pose.pose.orientation.z = float(self._custom_return_orientation[2])
        pose.pose.orientation.w = float(self._custom_return_orientation[3])
        return pose

    def _build_tcp_forward_pose(self, tcp_pose, distance):
        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = tcp_pose.header.frame_id
        o = tcp_pose.pose.orientation
        z_axis = self._quat_rotate([o.x, o.y, o.z, o.w], [0.0, 0.0, 1.0])
        out.pose.position.x = tcp_pose.pose.position.x + distance * z_axis[0]
        out.pose.position.y = tcp_pose.pose.position.y + distance * z_axis[1]
        out.pose.position.z = tcp_pose.pose.position.z + distance * z_axis[2]
        out.pose.orientation = tcp_pose.pose.orientation
        return out

    def _on_target_point(self, msg):
        self._last_target_xyz = [float(msg.point.x), float(msg.point.y), float(msg.point.z)]
        self._last_target_wall_time = time.monotonic()

    def _on_tcp_pose(self, msg):
        self._latest_tcp_pose = msg
        if self._initial_tcp_pose is None:
            self._initial_tcp_pose = msg
            self._logger.info("initial tcp pose captured")

    def _on_timer(self):
        self._publish_workspace_marker()
        now = time.monotonic()
        if self._last_target_xyz is None:
            return
        if self._last_target_wall_time is None or now - self._last_target_wall_time > self._lost_timeout_sec:
            self._reset_tracking_state()
            return

        if not self._kf_initialized:
            self._kf.reset(self._last_target_xyz)
            self._kf_initialized = True
            self._last_tick_wall_time = now
        else:
            dt = max(0.0, now - self._last_tick_wall_time)
            self._kf.predict(dt)
            self._kf.update(self._last_target_xyz)
            self._last_tick_wall_time = now

        pos = self._kf.position()
        vel = self._kf.velocity()
        object_xyz = [pos[i] + self._feedforward_sec * vel[i] for i in range(3)]
        object_xyz = self._workspace.clamp(object_xyz)
        self._latest_object_pose = self._build_grasp_pose(object_xyz)
        raw_grasp_pose = self._build_tcp_forward_pose(self._latest_object_pose, self._tcp_z_offset_m)
        raw_grasp_xyz = [
            float(raw_grasp_pose.pose.position.x),
            float(raw_grasp_pose.pose.position.y),
            float(raw_grasp_pose.pose.position.z),
        ]
        grasp_xyz = self._workspace.clamp(raw_grasp_xyz)
        self._latest_grasp_pose = self._build_grasp_pose(grasp_xyz)
        self._estimated_point_pub.publish(self._build_point_msg(object_xyz))
        if self._control_enabled:
            self._grasp_pose_pub.publish(self._latest_grasp_pose)

    def _reset_tracking_state(self):
        if not self._kf_initialized and self._latest_object_pose is None and self._latest_grasp_pose is None:
            return
        self._kf_initialized = False
        self._last_tick_wall_time = None
        self._latest_object_pose = None
        self._latest_grasp_pose = None

    def _build_grasp_pose(self, xyz):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._output_frame_id
        msg.pose.position.x = float(xyz[0])
        msg.pose.position.y = float(xyz[1])
        msg.pose.position.z = float(xyz[2])
        if self._latest_tcp_pose is not None:
            msg.pose.orientation = self._latest_tcp_pose.pose.orientation
        else:
            msg.pose.orientation.w = 1.0
        return msg

    def _build_point_msg(self, xyz):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._output_frame_id
        msg.point.x = float(xyz[0])
        msg.point.y = float(xyz[1])
        msg.point.z = float(xyz[2])
        return msg

    def _publish_workspace_marker(self):
        msg = Marker()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._output_frame_id
        msg.ns = "workspace"
        msg.id = 0
        msg.type = Marker.CUBE
        msg.action = Marker.ADD
        msg.pose.position.x = float(self._workspace.center[0])
        msg.pose.position.y = float(self._workspace.center[1])
        msg.pose.position.z = float(self._workspace.center[2])
        qx, qy, qz, qw = self._quat_from_euler_zyx(
            self._workspace.yaw_deg, self._workspace.pitch_deg, self._workspace.roll_deg
        )
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        msg.scale.x = 2.0 * float(self._workspace.half_extents[0])
        msg.scale.y = 2.0 * float(self._workspace.half_extents[1])
        msg.scale.z = 2.0 * float(self._workspace.half_extents[2])
        rgba = self._workspace_marker_rgba
        msg.color.r = float(rgba[0])
        msg.color.g = float(rgba[1])
        msg.color.b = float(rgba[2])
        msg.color.a = float(rgba[3])
        self._workspace_marker_pub.publish(msg)

    @staticmethod
    def _distance_between_pose(a, b):
        dx = float(a.pose.position.x) - float(b.pose.position.x)
        dy = float(a.pose.position.y) - float(b.pose.position.y)
        dz = float(a.pose.position.z) - float(b.pose.position.z)
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    @staticmethod
    def _quat_rotate(quat_xyzw, vec_xyz):
        qx, qy, qz, qw = [float(v) for v in quat_xyzw]
        vx, vy, vz = [float(v) for v in vec_xyz]
        tx = 2.0 * (qy * vz - qz * vy)
        ty = 2.0 * (qz * vx - qx * vz)
        tz = 2.0 * (qx * vy - qy * vx)
        return [
            vx + qw * tx + (qy * tz - qz * ty),
            vy + qw * ty + (qz * tx - qx * tz),
            vz + qw * tz + (qx * ty - qy * tx),
        ]

    @staticmethod
    def _quat_from_euler_zyx(yaw_deg, pitch_deg, roll_deg):
        yaw = math.radians(float(yaw_deg))
        pitch = math.radians(float(pitch_deg))
        roll = math.radians(float(roll_deg))
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        return [
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        ]


def main():
    rclpy.init()
    node = BasePolicyNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
