import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('humanoid_bringup')
    driver_config = os.path.join(bringup_share, 'config', 'ak70_can_driver.yaml')
    command_test_config = os.path.join(bringup_share, 'config', 'ak70_command_test.yaml')

    return LaunchDescription([
        Node(
            package='humanoid_control',
            executable='ak70_can_node',
            name='ak70_can_node',
            output='screen',
            parameters=[driver_config],
        ),
        Node(
            package='humanoid_control',
            executable='ak70_command_test_node',
            name='ak70_command_test_node',
            output='screen',
            parameters=[command_test_config],
        ),
    ])
