from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_PARAM_KEYS = (
    'cam1_topic',
    'cam2_topic',
    'cam2_camera_info_topic',
    'depth_img_topic',
    'weight_dir',
    'device',
    'inference_backend',
)


def generate_launch_description():
    package_name = 'fastfoundation'

    args = [
        DeclareLaunchArgument(
            'params_file',
            default_value='',
            description='可选：ROS2 参数 YAML 绝对路径；为空则不加载任何默认配置文件',
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='fastfoundation',
            description='命名空间',
        ),
        DeclareLaunchArgument(
            'node_name',
            default_value='fast_foundation_node',
            description='节点名',
        ),
    ]
    for key in _PARAM_KEYS:
        args.append(
            DeclareLaunchArgument(
                key,
                default_value='',
                description=f'覆盖参数 ros__parameters.{key}；空字符串表示不通过 launch 写入该键',
            )
        )

    def launch_setup(context):
        params = []
        pf = LaunchConfiguration('params_file').perform(context).strip()
        if pf:
            params.append(pf)
        overrides = {}
        for key in _PARAM_KEYS:
            v = LaunchConfiguration(key).perform(context).strip()
            if v:
                overrides[key] = v
        if overrides:
            params.append(overrides)
        return [
            Node(
                package=package_name,
                executable='fastfoundation',
                name=LaunchConfiguration('node_name'),
                namespace=LaunchConfiguration('namespace'),
                output='screen',
                emulate_tty=True,
                parameters=params,
            ),
        ]

    return LaunchDescription([*args, OpaqueFunction(function=launch_setup)])
