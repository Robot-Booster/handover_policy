from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


_OVERRIDABLE_KEYS = (
    "depth_topic",
    "depth_camera_info_topic",
    "seg_mask_topic",
    "cloud_topic",
    "centroid_topic",
    "workspace_marker_topic",
    "target_frame",
)


def _build_node(context):
    namespace = LaunchConfiguration("namespace").perform(context)
    config_file = LaunchConfiguration("config_file")

    # 仅在命令行显式给非空值时覆盖 YAML
    # Only override YAML when CLI value is non-empty.
    overrides = {}
    for key in _OVERRIDABLE_KEYS:
        value = LaunchConfiguration(key).perform(context)
        if value != "":
            overrides[key] = value

    return [
        Node(
            package="pc_processor",
            executable="perception_node",
            namespace=namespace,
            output="screen",
            parameters=[config_file, overrides],
        )
    ]


def generate_launch_description():
    args = [
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("pc_processor"), "config", "perception.yaml"]
            ),
        ),
    ]
    for key in _OVERRIDABLE_KEYS:
        args.append(DeclareLaunchArgument(key, default_value=""))
    args.append(OpaqueFunction(function=_build_node))
    return LaunchDescription(args)
