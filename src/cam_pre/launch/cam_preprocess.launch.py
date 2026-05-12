from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def _launch_cam_pre(context):
    ns = LaunchConfiguration("namespace").perform(context).strip()
    cam = LaunchConfiguration("camera_name").perform(context).strip()

    node_name = cam if cam else "camera_pre_node"

    node = Node(
        package="cam_pre",
        executable="camera_pre",
        name=node_name,
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
                    "Outer namespace (RealSense-style leading segment), e.g. 'camera'. "
                    "Empty = node not pushed under extra ns."
                ),
            ),
            DeclareLaunchArgument(
                "camera_name",
                default_value="",
                description=(
                    "Camera id as node short name, e.g. 'd455'. Full node: /namespace/camera_name. "
                    "Empty = fallback name 'camera_pre_node'."
                ),
            ),
            DeclareLaunchArgument(
                "config_file",
                description="Path to YAML parameter file (global /** ros__parameters).",
            ),
            OpaqueFunction(function=_launch_cam_pre),
        ]
    )
