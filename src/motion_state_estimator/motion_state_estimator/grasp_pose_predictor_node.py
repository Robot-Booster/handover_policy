import json
import math
import time

try:
    from geometry_msgs.msg import PointStamped, PoseStamped
except ModuleNotFoundError:
    class _Header:
        def __init__(self):
            self.stamp = None
            self.frame_id = ""

    class _Point:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0

    class _Quaternion:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
            self.w = 1.0

    class _Pose:
        def __init__(self):
            self.position = _Point()
            self.orientation = _Quaternion()

    class PointStamped:  # type: ignore[no-redef]
        def __init__(self):
            self.header = _Header()
            self.point = _Point()

    class PoseStamped:  # type: ignore[no-redef]
        def __init__(self):
            self.header = _Header()
            self.pose = _Pose()

try:
    from visualization_msgs.msg import Marker
except ModuleNotFoundError:
    class _Color:
        def __init__(self):
            self.r = 0.0
            self.g = 0.0
            self.b = 0.0
            self.a = 0.0

    class Marker:  # type: ignore[no-redef]
        CUBE = 1
        ADD = 0

        def __init__(self):
            self.header = _Header()
            self.ns = ""
            self.id = 0
            self.type = 0
            self.action = 0
            self.pose = _Pose()
            self.scale = _Point()
            self.color = _Color()

try:
    import rclpy
    from rclpy.node import Node
except ModuleNotFoundError:
    rclpy = None

    class Node:  # type: ignore[no-redef]
        pass

try:
    from tf2_ros import Buffer, TransformException, TransformListener
except ModuleNotFoundError:
    Buffer = None
    TransformListener = None

    class TransformException(Exception):  # type: ignore[no-redef]
        pass


class _LinearCvKalmanFilter:
    def __init__(self, process_noise_pos, process_noise_vel, measure_noise_pos):
        self._q_pos = float(process_noise_pos)
        self._q_vel = float(process_noise_vel)
        self._r_pos = float(measure_noise_pos)
        if self._q_pos < 0.0 or self._q_vel < 0.0 or self._r_pos <= 0.0:
            raise ValueError("invalid kalman noise parameters")
        self._x = [[0.0, 0.0] for _ in range(3)]
        self._p = [[[1.0, 0.0], [0.0, 1.0]] for _ in range(3)]

    def reset(self, p_xyz):
        for i in range(3):
            self._x[i][0] = float(p_xyz[i])
            self._x[i][1] = 0.0
            self._p[i] = [[1.0, 0.0], [0.0, 1.0]]

    def predict(self, dt):
        dt = float(dt)
        if dt < 0.0:
            raise ValueError("dt must be >= 0")
        dt2 = dt * dt
        for i in range(3):
            p00, p01 = self._p[i][0]
            p10, p11 = self._p[i][1]
            self._x[i][0] += dt * self._x[i][1]
            p00_new = p00 + dt * (p01 + p10) + dt2 * p11 + self._q_pos
            p01_new = p01 + dt * p11
            p10_new = p10 + dt * p11
            p11_new = p11 + self._q_vel
            self._p[i] = [[p00_new, p01_new], [p10_new, p11_new]]

    def update(self, z_xyz):
        for i in range(3):
            z = float(z_xyz[i])
            p00, p01 = self._p[i][0]
            p10, p11 = self._p[i][1]
            y = z - self._x[i][0]
            s = p00 + self._r_pos
            if s <= 1e-12:
                raise ValueError("invalid innovation covariance")
            k0 = p00 / s
            k1 = p10 / s
            self._x[i][0] += k0 * y
            self._x[i][1] += k1 * y
            self._p[i] = [
                [(1.0 - k0) * p00, (1.0 - k0) * p01],
                [p10 - k1 * p00, p11 - k1 * p01],
            ]

    def position(self):
        return [self._x[i][0] for i in range(3)]

    def velocity(self):
        return [self._x[i][1] for i in range(3)]


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
        order = data.get("euler_deg_order", "ZYX")
        if order != "ZYX":
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


class GraspPosePredictorNode(Node):
    def __init__(self):
        super().__init__("grasp_pose_predictor")
        self._logger = self.get_logger()
        self._state = "INIT"
        self._locked_orientation = None
        self._last_target_xyz = None
        self._last_target_wall_time = None
        self._last_tick_wall_time = None
        self._kf_initialized = False

        input_point_topic = self.declare_parameter("input_point_topic", "~/target_point").value
        input_tcp_pose_topic = self.declare_parameter("input_tcp_pose_topic", "~/tcp_pose").value
        output_grasp_pose_topic = self.declare_parameter("output_grasp_pose_topic", "~/grasp_pose").value
        output_debug_point_topic = self.declare_parameter("output_debug_point_topic", "~/debug_point").value
        self._workspace_marker_topic = self.declare_parameter(
            "workspace_marker_topic", "~/workspace_marker"
        ).value
        self._output_frame_id = self.declare_parameter("output_frame_id", "base_link").value
        self._base_frame = str(self.declare_parameter("base_frame", "base_link").value)
        self._ee_frame = str(self.declare_parameter("ee_frame", "tool0").value)
        self._tcp_z_offset_m = float(self.declare_parameter("tcp_z_offset_m", 0.0).value)
        update_hz = float(self.declare_parameter("update_hz", 20.0).value)
        self._feedforward_sec = float(self.declare_parameter("feedforward_sec", 0.0).value)
        self._lost_timeout_sec = float(self.declare_parameter("lost_timeout_sec", 0.25).value)
        self._enable_debug_point = bool(self.declare_parameter("enable_debug_point", False).value)
        workspace_box_json_path = str(self.declare_parameter("workspace_box_json_path", "").value).strip()
        if Buffer is None or TransformListener is None:
            self.get_logger().error("tf2_ros is not available")
            raise RuntimeError("tf2_ros is not available")
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        if not workspace_box_json_path:
            self.get_logger().error("workspace_box_json_path is empty")
            raise RuntimeError("workspace_box_json_path is empty")
        try:
            self._workspace = _WorkspaceObb.from_json(workspace_box_json_path)
        except Exception as exc:
            self.get_logger().error(f"invalid workspace json: {exc}")
            raise RuntimeError("invalid workspace json") from exc

        self._kf = _LinearCvKalmanFilter(
            process_noise_pos=float(self.declare_parameter("process_noise_pos", 1.0e-4).value),
            process_noise_vel=float(self.declare_parameter("process_noise_vel", 1.0e-2).value),
            measure_noise_pos=float(self.declare_parameter("measure_noise_pos", 1.0e-3).value),
        )

        self._grasp_pose_pub = self.create_publisher(PoseStamped, output_grasp_pose_topic, 10)
        self._workspace_marker_pub = self.create_publisher(Marker, self._workspace_marker_topic, 10)
        self._debug_point_pub = (
            self.create_publisher(PointStamped, output_debug_point_topic, 10) if self._enable_debug_point else None
        )
        self._workspace_marker_rgba = [
            float(v)
            for v in self.declare_parameter("workspace_marker_rgba", [0.0, 0.0, 1.0, 0.25]).value
        ]
        if len(self._workspace_marker_rgba) != 4:
            raise RuntimeError("workspace_marker_rgba must have 4 elements")
        self._logger.info(
            "Node parameters: "
            f"input_point_topic={input_point_topic}, "
            f"input_tcp_pose_topic={input_tcp_pose_topic}, "
            f"output_grasp_pose_topic={output_grasp_pose_topic}, "
            f"output_debug_point_topic={output_debug_point_topic}, "
            f"workspace_marker_topic={self._workspace_marker_topic}, "
            f"output_frame_id={self._output_frame_id}, "
            f"base_frame={self._base_frame}, "
            f"ee_frame={self._ee_frame}, "
            f"update_hz={update_hz}, "
            f"process_noise_pos={self._kf._q_pos}, "
            f"process_noise_vel={self._kf._q_vel}, "
            f"measure_noise_pos={self._kf._r_pos}, "
            f"feedforward_sec={self._feedforward_sec}, "
            f"lost_timeout_sec={self._lost_timeout_sec}, "
            f"tcp_z_offset_m={self._tcp_z_offset_m}, "
            f"enable_debug_point={self._enable_debug_point}, "
            f"workspace_box_json_path={workspace_box_json_path}, "
            f"workspace_marker_rgba={self._workspace_marker_rgba}"
        )
        self.create_subscription(PointStamped, input_point_topic, self._on_target_point, 10)
        self.create_subscription(PoseStamped, input_tcp_pose_topic, self._on_tcp_pose, 10)
        self.create_timer(1.0 / max(update_hz, 1.0), self._on_timer)

    def _on_target_point(self, msg):
        self._last_target_xyz = [float(msg.point.x), float(msg.point.y), float(msg.point.z)]
        self._last_target_wall_time = time.monotonic()

    def _on_tcp_pose(self, msg):
        if self._locked_orientation is None:
            o = msg.pose.orientation
            locked = type(o)()
            locked.x = float(o.x)
            locked.y = float(o.y)
            locked.z = float(o.z)
            locked.w = float(o.w)
            self._locked_orientation = locked

    def _on_timer(self):
        self._tick()

    def _set_state(self, new_state):
        if new_state == self._state:
            return
        if new_state == "TRACKING":
            self._logger.info("Tracking acquired.")
        elif new_state == "LOST":
            self._logger.warn("Tracking lost.")
        self._state = new_state

    def _tick(self, now_sec=None):
        self._publish_workspace_marker()
        now = time.monotonic() if now_sec is None else float(now_sec)
        if self._locked_orientation is None:
            self._set_state("INIT")
            return
        if self._last_target_xyz is None or self._last_target_wall_time is None:
            if self._state != "LOST":
                self._set_state("INIT")
            return
        if now - self._last_target_wall_time > self._lost_timeout_sec:
            self._set_state("LOST")
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
        out_xyz = [pos[i] + self._feedforward_sec * vel[i] for i in range(3)]
        try:
            out_xyz = self._apply_tcp_z_offset(out_xyz)
        except Exception as exc:
            self.get_logger().warn(f"skip publish this cycle due to tf lookup failure: {exc}")
            return
        out_xyz = self._workspace.clamp(out_xyz)
        self._publish_grasp_pose(out_xyz)
        self._publish_debug_point(out_xyz)
        self._set_state("TRACKING")

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

    def _lookup_ee_z_in_base(self):
        tf = self._tf_buffer.lookup_transform(self._base_frame, self._ee_frame, rclpy.time.Time())
        q = tf.transform.rotation
        return self._quat_rotate([q.x, q.y, q.z, q.w], [0.0, 0.0, 1.0])

    def _apply_tcp_z_offset(self, point_xyz):
        z_world = self._lookup_ee_z_in_base()
        return [float(point_xyz[i]) + self._tcp_z_offset_m * float(z_world[i]) for i in range(3)]

    def _publish_grasp_pose(self, point_xyz):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._output_frame_id
        msg.pose.position.x = point_xyz[0]
        msg.pose.position.y = point_xyz[1]
        msg.pose.position.z = point_xyz[2]
        msg.pose.orientation = self._locked_orientation
        self._grasp_pose_pub.publish(msg)

    def _publish_debug_point(self, point_xyz):
        if not self._enable_debug_point or self._debug_point_pub is None:
            return
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._output_frame_id
        msg.point.x = point_xyz[0]
        msg.point.y = point_xyz[1]
        msg.point.z = point_xyz[2]
        self._debug_point_pub.publish(msg)

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

    def _publish_workspace_marker(self):
        msg = Marker()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._base_frame
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


def main():
    if rclpy is None:
        raise RuntimeError("rclpy is not available")
    rclpy.init()
    node = GraspPosePredictorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
