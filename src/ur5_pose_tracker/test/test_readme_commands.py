from pathlib import Path


def test_readme_has_required_run_commands():
    readme_file = Path(__file__).resolve().parents[1] / "README.md"
    assert readme_file.is_file()

    content = readme_file.read_text(encoding="utf-8")
    assert "source .venv/bin/activate" in content
    assert (
        "ros2 launch ur5_pose_tracker pose_tracker.launch.py "
        "params_file:=src/ur5_pose_tracker/config/pose_tracker.yaml"
    ) in content
    assert "ros2 topic pub" in content
    assert "PoseStamped" in content
    assert "-r 20" in content
