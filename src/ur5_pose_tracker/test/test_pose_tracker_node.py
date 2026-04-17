from pathlib import Path
from types import SimpleNamespace
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ur5_pose_tracker.pose_tracker_node import RTDEServoNode


class _DummyControl:
    def __init__(self):
        self.calls = []
        self.stop_calls = 0

    def servoL(self, target, speed, acceleration, dt, lookahead_time, gain):
        self.calls.append((target, speed, acceleration, dt, lookahead_time, gain))

    def servoStop(self):
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


def test_validate_frame_accept_and_reject():
    node = RTDEServoNode(
        accepted_frame_ids=["base", "tool0"],
        rtde_control=_DummyControl(),
        rtde_receive=_DummyReceive(),
        control_hz=20.0,
        use_ros=False,
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
        use_ros=False,
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
        use_ros=False,
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
        use_ros=False,
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
        use_ros=False,
    )
    dummy_ros_node = _DummyRosNode()
    node._ros_node = dummy_ros_node
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
