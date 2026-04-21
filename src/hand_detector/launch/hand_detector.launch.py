from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _build_node(context):
    namespace = LaunchConfiguration("namespace")
    config_file = LaunchConfiguration("config_file")
    confidence = LaunchConfiguration("confidence")
    model_path = LaunchConfiguration("model_path")
    image_topic = LaunchConfiguration("image_topic")
    mask_topic = LaunchConfiguration("mask_topic")

    overrides = {}
    confidence_value = confidence.perform(context).strip()
    model_path_value = model_path.perform(context).strip()
    if confidence_value:
        overrides["confidence"] = float(confidence_value)
    if model_path_value:
        overrides["model_path"] = model_path_value

    params = [config_file]
    if overrides:
        params.append(overrides)

    return [
        Node(
            package="hand_detector",
            executable="hand_detector_node",
            name="hand_detector",
            namespace=namespace,
            output="screen",
            parameters=params,
            remappings=[
                ("~/image_raw", image_topic),
                ("~/hand_mask", mask_topic),
            ],
        )
    ]


def generate_launch_description():

    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Node namespace.",
    )
    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("hand_detector"), "config", "config.yaml"]
        ),
        description="Path to config.yaml.",
    )
    confidence_arg = DeclareLaunchArgument(
        "confidence",
        default_value="",
        description="Confidence threshold override.",
    )
    model_path_arg = DeclareLaunchArgument(
        "model_path",
        default_value="",
        description="Model weights absolute path override.",
    )
    image_topic_arg = DeclareLaunchArgument(
        "image_topic",
        default_value="~/image_raw",
        description="Input image topic remap target.",
    )
    mask_topic_arg = DeclareLaunchArgument(
        "mask_topic",
        default_value="~/hand_mask",
        description="Output mask topic remap target.",
    )

    return LaunchDescription(
        [
            namespace_arg,
            config_file_arg,
            confidence_arg,
            model_path_arg,
            image_topic_arg,
            mask_topic_arg,
            OpaqueFunction(function=_build_node),
        ]
    )
