from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 레거시 파일명 유지: 현재는 토픽 기반 드라이버 노드를 실행한다.
        Node(
            package='humanoid_control',
            executable='ak70_can_node',
            name='ak70_can_node',
            output='screen',
            parameters=[
                {'can_interface': 'can0'},
                {'motor_id': 0x05},
                {'control_period_sec': 0.02},
                {'kp': 12.0},
                {'kd': 1.0},
                {'command_timeout_sec': 0.5},
                {'startup_zero_command_repeat': 50},
                {'startup_zero_command_dt': 0.01},
            ],
        )
    ])
