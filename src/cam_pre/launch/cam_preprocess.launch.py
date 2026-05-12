from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def _launch_with_optional_namespace(context):
    """Empty namespace → node at /; non-empty → under /<namespace>/."""
    ns = LaunchConfiguration("namespace").perform(context).strip()
    node = Node(
        package="cam_pre",
        executable="camera_pre",
        name="camera_pre_node",
        parameters=[LaunchConfiguration("config_file")],
        output="screen",
    )
    if ns:
        return [GroupAction([PushRosNamespace(ns), node])]
    return [node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description=(
                    "ROS namespace for this node (e.g. cam_left). "
                    "Empty = root. Only affects node name and ~/ topic relative paths."
                ),
            ),
            DeclareLaunchArgument(
                "config_file",
                description="Path to YAML parameter file (global /** ros__parameters).",
            ),
            OpaqueFunction(function=_launch_with_optional_namespace),
        ]
    )
