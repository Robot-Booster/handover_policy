from pathlib import Path
from types import SimpleNamespace
import sys

import pytest
import rclpy

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ur5_pose_tracker.pose_tracker_node import RTDEServoNode


@pytest.fixture(scope="module", autouse=True)
def _rclpy_module_context():
    """RTDEServoNode 继承 rclpy.Node，单测需先初始化 rclpy。"""
    import os

    log_dir = Path(__file__).resolve().parent / ".ros_test_logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    prev_ros_log = os.environ.get("ROS_LOG_DIR")
    os.environ["ROS_LOG_DIR"] = str(log_dir)
    try:
        rclpy.init()
        yield
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        if prev_ros_log is None:
            os.environ.pop("ROS_LOG_DIR", None)
        else:
            os.environ["ROS_LOG_DIR"] = prev_ros_log


class _DummyControl:
    def __init__(self):
        self.calls = []
        self.stop_calls = 0

    def servoL(self, target, speed, acceleration, dt, lookahead_time, gain):
        self.calls.append((target, speed, acceleration, dt, lookahead_time, gain))

    def servoStop(self, _a=10.0):
        self.stop_calls += 1


class _DummyReceive:
    def getActualTCPPose(self):
        return [0.0] * 6


class _DummyLogger:
    def __init__(self):
        self.infos = []
        self.warnings = []
        self.errors = []

    def info(self, message):
        self.infos.append(message)

    def warning(self, message):
        self.warnings.append(message)

    def error(self, message):
        self.errors.append(message)


class _DummyParameter:
    def __init__(self, value):
        self.value = value


class _DummyRosNode:
    def __init__(self):
        self._params = {}

    def declare_parameter(self, name, default_value):
        self._params.setdefault(name, default_value)

    def get_parameter(self, name):
        return _DummyParameter(self._params[name])

    def set_parameter(self, name, value):
        self._params[name] = value


def _pose_msg(frame_id, x):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame_id),
        pose=SimpleNamespace(
            position=SimpleNamespace(x=x, y=0.2, z=0.3),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
    )


def _build_pose_stamped_class():
    class _PoseStamped:
        def __init__(self):
            self.header = SimpleNamespace(stamp=None, frame_id="")
            self.pose = SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            )

    return _PoseStamped


def test_validate_frame_accept_and_reject():
    node = RTDEServoNode(
        accepted_frame_ids=["base", "tool0"],
        rtde_control=_DummyControl(),
        rtde_receive=_DummyReceive(),
        control_hz=20.0,
    )

    assert node._validate_frame("base") is True
    assert node._validate_frame("tool0") is True
    assert node._validate_frame("camera") is False
    assert node._validate_frame("") is False


def test_latest_only_overwrites_old_target():
    control = _DummyControl()
    node = RTDEServoNode(
        accepted_frame_ids=["base"],
        rtde_control=control,
        rtde_receive=_DummyReceive(),
        control_hz=20.0,
    )

    node._on_pose_msg(_pose_msg("base", 0.1))
    node._on_pose_msg(_pose_msg("base", 0.9))

    node.run_control_loop(max_steps=1)

    assert len(control.calls) == 1
    assert control.calls[0][0][0] == pytest.approx(0.9)


def test_run_control_loop_triggers_safe_stop_on_timeout(monkeypatch):
    control = _DummyControl()
    logger = _DummyLogger()
    node = RTDEServoNode(
        accepted_frame_ids=["base"],
        rtde_control=control,
        rtde_receive=_DummyReceive(),
        control_hz=20.0,
        logger=logger,
    )
    node._pose_timeout_sec = 0.1
    node._latest_target = [0.1, 0.2, 0.3, 0.0, 0.0, 0.0]
    node._latest_target_time = 0.0

    monkeypatch.setattr("ur5_pose_tracker.pose_tracker_node.time.monotonic", lambda: 10.0)

    node.run_control_loop(max_steps=1)

    assert len(control.calls) == 0
    assert node._latest_target is None
    assert control.stop_calls >= 1
    assert any("Pose timeout" in message for message in logger.warnings)


def test_run_control_loop_handles_send_servo_exception(monkeypatch):
    control = _DummyControl()
    logger = _DummyLogger()
    node = RTDEServoNode(
        accepted_frame_ids=["base"],
        rtde_control=control,
        rtde_receive=_DummyReceive(),
        control_hz=20.0,
        logger=logger,
    )
    node._latest_target = [0.1, 0.2, 0.3, 0.0, 0.0, 0.0]
    node._latest_target_time = 10.0
    node._pose_timeout_sec = 5.0
    monkeypatch.setattr("ur5_pose_tracker.pose_tracker_node.time.monotonic", lambda: 10.0)
    node._send_servo = lambda _target: (_ for _ in ()).throw(RuntimeError("boom"))

    node.run_control_loop(max_steps=1)

    assert node._latest_target is None
    assert control.stop_calls >= 1
    assert any("servoL failed: boom" in message for message in logger.errors)


def test_load_ros_parameters_updates_runtime_config():
    node = RTDEServoNode(
        accepted_frame_ids=["base"],
        rtde_control=_DummyControl(),
        rtde_receive=_DummyReceive(),
    )
    dummy_ros_node = _DummyRosNode()
    node._param_node = dummy_ros_node
    node._declare_ros_parameters()

    dummy_ros_node.set_parameter("robot_ip", "192.168.0.2")
    dummy_ros_node.set_parameter("input_topic", "/camera_pose")
    dummy_ros_node.set_parameter("control_hz", 50.0)
    dummy_ros_node.set_parameter("accepted_frame_ids", ["base", "tool0"])
    dummy_ros_node.set_parameter("pose_timeout_sec", 0.3)
    dummy_ros_node.set_parameter("servo_speed", 0.5)
    dummy_ros_node.set_parameter("servo_acceleration", 0.8)
    dummy_ros_node.set_parameter("servo_lookahead_time", 0.2)
    dummy_ros_node.set_parameter("servo_gain", 350)

    node._load_ros_parameters()

    assert node._robot_ip == "192.168.0.2"
    assert node._input_topic == "/camera_pose"
    assert node._control_hz == pytest.approx(50.0)
    assert node._accepted_frame_ids == {"base", "tool0"}
    assert node._pose_timeout_sec == pytest.approx(0.3)
    assert node._speed == pytest.approx(0.5)
    assert node._acceleration == pytest.approx(0.8)
    assert node._lookahead_time == pytest.approx(0.2)
    assert node._gain == pytest.approx(350.0)


def test_publish_tcp_pose_success_path(monkeypatch):
    control = _DummyControl()
    logger = _DummyLogger()
    published = []
    node = RTDEServoNode(
        accepted_frame_ids=["base_link"],
        rtde_control=control,
        rtde_receive=_DummyReceive(),
        control_hz=20.0,
        logger=logger,
    )
    node._tcp_pose_publisher = SimpleNamespace(publish=lambda msg: published.append(msg))
    node._rtde_receive = SimpleNamespace(
        getActualTCPPose=lambda: [0.1, -0.2, 0.3, 0.0, 0.0, 0.0]
    )
    node._now_ros_time = lambda: SimpleNamespace(
        to_msg=lambda: SimpleNamespace(sec=1, nanosec=2)
    )
    monkeypatch.setattr(
        "ur5_pose_tracker.pose_tracker_node.PoseStamped", _build_pose_stamped_class()
    )

    node._publish_tcp_pose()

    assert len(published) == 1
    msg = published[0]
    assert msg.header.frame_id == "base_link"
    assert msg.pose.position.x == pytest.approx(0.1)
    assert msg.pose.position.y == pytest.approx(-0.2)
    assert msg.pose.position.z == pytest.approx(0.3)
    assert msg.pose.orientation.x == pytest.approx(0.0)
    assert msg.pose.orientation.y == pytest.approx(0.0)
    assert msg.pose.orientation.z == pytest.approx(0.0)
    assert msg.pose.orientation.w == pytest.approx(1.0)


def test_publish_tcp_pose_skip_on_rtde_error(monkeypatch):
    node = RTDEServoNode(
        accepted_frame_ids=["base_link"],
        rtde_control=_DummyControl(),
        rtde_receive=_DummyReceive(),
        control_hz=20.0,
        logger=_DummyLogger(),
    )
    node._tcp_pose_publisher = SimpleNamespace(
        publish=lambda _msg: (_ for _ in ()).throw(AssertionError("should not publish"))
    )
    node._rtde_receive = SimpleNamespace(
        getActualTCPPose=lambda: (_ for _ in ()).throw(RuntimeError("rtde down"))
    )
    monkeypatch.setattr(
        "ur5_pose_tracker.pose_tracker_node.PoseStamped", _build_pose_stamped_class()
    )

    node._publish_tcp_pose()

    assert any(
        "Failed to read tcp pose: rtde down" in message
        for message in node._logger.warnings
    )


def test_run_control_loop_publishes_tcp_pose_every_step(monkeypatch):
    node = RTDEServoNode(
        accepted_frame_ids=["base_link"],
        rtde_control=_DummyControl(),
        rtde_receive=_DummyReceive(),
        control_hz=20.0,
    )
    publish_count = {"value": 0}
    monkeypatch.setattr("ur5_pose_tracker.pose_tracker_node.time.sleep", lambda _dt: None)
    node._publish_tcp_pose = lambda: publish_count.__setitem__(
        "value", publish_count["value"] + 1
    )

    node.run_control_loop(max_steps=3)

    assert publish_count["value"] == 3


def test_rotvec_to_quat_zero_rotation():
    node = RTDEServoNode(
        accepted_frame_ids=["base_link"],
        rtde_control=_DummyControl(),
        rtde_receive=_DummyReceive(),
    )
    quat = node._rotvec_to_quat(0.0, 0.0, 0.0)

    assert quat[0] == pytest.approx(0.0)
    assert quat[1] == pytest.approx(0.0)
    assert quat[2] == pytest.approx(0.0)
    assert quat[3] == pytest.approx(1.0)
