from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _build_node(context):
    namespace = LaunchConfiguration("namespace").perform(context)
    config_file = LaunchConfiguration("config_file")

    # 仅在命令行显式给值时覆盖 YAML；缺省不覆盖
    # Only override YAML when CLI value is explicitly provided.
    overrides = {}
    for key in [
        "depth_topic",
        "depth_camera_info_topic",
        "mask_topic",
        "mask_camera_info_topic",
        "cloud_topic",
        "align_frame",
    ]:
        value = LaunchConfiguration(key).perform(context)
        if value != "":
            overrides[key] = value

    return [
        Node(
            package="pc_processor",
            executable="depth_to_cloud_node",
            namespace=namespace,
            output="screen",
            parameters=[config_file, overrides],
        )
    ]


def generate_launch_description():
    from launch.actions import OpaqueFunction

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="",
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("pc_processor"), "config", "depth_to_cloud.yaml"]
                ),
            ),
            DeclareLaunchArgument("depth_topic", default_value=""),
            DeclareLaunchArgument("depth_camera_info_topic", default_value=""),
            DeclareLaunchArgument("mask_topic", default_value=""),
            DeclareLaunchArgument("mask_camera_info_topic", default_value=""),
            DeclareLaunchArgument("cloud_topic", default_value=""),
            DeclareLaunchArgument("align_frame", default_value=""),
            OpaqueFunction(function=_build_node),
        ]
    )
