from pathlib import Path
from types import SimpleNamespace
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ur5_pose_tracker.pose_tracker_node import RTDEServoNode


class _DummyControl:
    def __init__(self):
        self.calls = []

    def servoL(self, target, speed, acceleration, dt, lookahead_time, gain):
        self.calls.append((target, speed, acceleration, dt, lookahead_time, gain))

    def servoStop(self):
        pass


class _DummyReceive:
    def getActualTCPPose(self):
        return [0.0] * 6


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
        expected_frame_id="base",
        rtde_control=_DummyControl(),
        rtde_receive=_DummyReceive(),
        control_hz=20.0,
        use_ros=False,
    )

    assert node._validate_frame("base") is True
    assert node._validate_frame("tool0") is False
    assert node._validate_frame("") is False


def test_latest_only_overwrites_old_target():
    control = _DummyControl()
    node = RTDEServoNode(
        expected_frame_id="base",
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
