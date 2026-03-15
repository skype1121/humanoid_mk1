import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    config_file = LaunchConfiguration('config_file').perform(context)
    launch_ui = LaunchConfiguration('launch_ui').perform(context).strip().lower()

    nodes = []
    if launch_ui in ('true', '1', 'yes', 'on'):
        nodes.append(
            Node(
                package='humanoid_control',
                executable='ak70_control_panel',
                name='ak70_control_panel',
                output='screen',
                parameters=[config_file],
            )
        )

    # 드라이버와 UI가 같은 YAML을 읽도록 유지해서 모터 개수 불일치를 막는다.
    # UI를 먼저 기동해서 CAN/드라이버 문제와 무관하게 패널이 먼저 뜨도록 둔다.
    nodes.append(
        Node(
            package='humanoid_control',
            executable='ak70_driver_node',
            name='ak70_driver_node',
            output='screen',
            parameters=[config_file],
        )
    )

    return nodes


def generate_launch_description():
    default_config_path = os.path.join(
        get_package_share_directory('humanoid_control'),
        'config',
        'ak70_motors.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config_path,
            description='AK70 driver YAML config file',
        ),
        DeclareLaunchArgument(
            'launch_ui',
            default_value='false',
            description='Launch AK70 Tk control panel',
        ),
        OpaqueFunction(function=launch_setup),
    ])
