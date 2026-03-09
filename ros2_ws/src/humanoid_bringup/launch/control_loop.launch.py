import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    control_loop_config = os.path.join(
        get_package_share_directory('humanoid_bringup'),
        'config',
        'control_loop_node.yaml',
    )

    return LaunchDescription([
        Node(
            package='humanoid_control',
            executable='control_loop_node',
            name='control_loop_node',
            output='screen',
            parameters=[control_loop_config],
        ),
        Node(
            package='humanoid_bringup',
            executable='control_loop_subscriber_node',
            name='control_loop_subscriber_node',
            output='screen',
        ),
    ])
