import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
        Node(
            package='humanoid_control',
            executable='ak70_driver_node',
            name='ak70_driver_node',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
        )
    ])
