from pathlib import Path


def test_launch_file_exists():
    launch_file = Path(__file__).resolve().parents[1] / "launch" / "pose_tracker.launch.py"
    assert launch_file.is_file()


def test_config_file_exists():
    config_file = Path(__file__).resolve().parents[1] / "config" / "pose_tracker.yaml"
    assert config_file.is_file()
