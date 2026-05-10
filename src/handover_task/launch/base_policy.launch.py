from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _build_optional_remap(src_name, cli_value):
    value = (cli_value or "").strip()
    if value == "":
        return None
    return (src_name, value)


def _build_node(context):
    config_file = LaunchConfiguration("config_file")
    namespace = LaunchConfiguration("namespace").perform(context)
    topic_specs = [
        ("input_point_topic", "~/target_point"),
        ("input_tcp_pose_topic", "~/tcp_pose"),
        ("output_grasp_pose_topic", "~/grasp_pose"),
        ("workspace_marker_topic", "~/workspace_marker"),
    ]
    remappings = []
    overrides = {}
    for arg_name, src_name in topic_specs:
        cli_value = LaunchConfiguration(arg_name).perform(context)
        remap = _build_optional_remap(src_name, cli_value)
        if remap is not None:
            remappings.append(remap)
            overrides[arg_name] = remap[1]

    return [
        Node(
            package="handover_task",
            executable="base_policy",
            name="base_policy",
            namespace=namespace,
            output="screen",
            parameters=[config_file, overrides],
            remappings=remappings,
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument(
                "config_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("handover_task"),
                        "config",
                        "base_policy.yaml",
                    ]
                ),
                description="Path to base policy parameter file.",
            ),
            DeclareLaunchArgument("input_point_topic", default_value=""),
            DeclareLaunchArgument("input_tcp_pose_topic", default_value=""),
            DeclareLaunchArgument("output_grasp_pose_topic", default_value=""),
            DeclareLaunchArgument("workspace_marker_topic", default_value=""),
            OpaqueFunction(function=_build_node),
        ]
    )
