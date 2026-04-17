from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    input_topic = LaunchConfiguration("input_topic")
    node_executable = LaunchConfiguration("node_executable")

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("ur5_pose_tracker"), "config", "pose_tracker.yaml"]
        ),
        description="Path to the pose tracker parameter file.",
    )
    input_topic_arg = DeclareLaunchArgument(
        "input_topic",
        default_value="~/ur_target_pose",
        description="Input pose topic for tracking.",
    )
    node_executable_arg = DeclareLaunchArgument(
        "node_executable",
        default_value="pose_tracker_node",
        description="Node executable name, implemented in Task 3.",
    )

    pose_tracker_node = Node(
        package="ur5_pose_tracker",
        executable=node_executable,
        name="ur5_pose_tracker",
        output="screen",
        parameters=[params_file],
        remappings=[("~/ur_target_pose", input_topic)],
    )

    return LaunchDescription(
        [params_file_arg, input_topic_arg, node_executable_arg, pose_tracker_node]
    )
