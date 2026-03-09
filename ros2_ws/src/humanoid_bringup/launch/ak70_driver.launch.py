import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    driver_config = os.path.join(
        get_package_share_directory('humanoid_bringup'),
        'config',
        'ak70_can_driver.yaml',
    )

    return LaunchDescription([
        Node(
            package='humanoid_control',
            executable='ak70_can_node',
            name='ak70_can_node',
            output='screen',
            parameters=[driver_config],
        ),
    ])
