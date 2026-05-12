from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                description="Path to YAML parameter file (global /** ros__parameters).",
            ),
            Node(
                package="camera",
                executable="camera_pre",
                name="camera_pre_node",
                parameters=[LaunchConfiguration("config_file")],
                output="screen",
            ),
        ]
    )
