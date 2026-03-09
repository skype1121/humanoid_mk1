#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class AK70CommandTestNode(Node):
    def __init__(self) -> None:
        super().__init__('ak70_command_test_node')

        self.declare_parameter('mode', 'hold')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('hold_target_rad', 0.2)
        self.declare_parameter('step_target_rad', 0.5)
        self.declare_parameter('step_hold_sec', 2.0)
        self.declare_parameter('sine_amplitude_rad', 0.2)
        self.declare_parameter('sine_offset_rad', 0.0)
        self.declare_parameter('sine_frequency_hz', 0.2)

        self.mode = str(self.get_parameter('mode').value).strip().lower()
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.hold_target_rad = float(self.get_parameter('hold_target_rad').value)
        self.step_target_rad = float(self.get_parameter('step_target_rad').value)
        self.step_hold_sec = float(self.get_parameter('step_hold_sec').value)
        self.sine_amplitude_rad = float(self.get_parameter('sine_amplitude_rad').value)
        self.sine_offset_rad = float(self.get_parameter('sine_offset_rad').value)
        self.sine_frequency_hz = float(self.get_parameter('sine_frequency_hz').value)

        if self.publish_rate_hz <= 0.0:
            self.publish_rate_hz = 20.0
        if self.step_hold_sec <= 0.0:
            self.step_hold_sec = 2.0

        if self.mode not in ('hold', 'step', 'sine'):
            self.get_logger().warn(f"Unsupported mode '{self.mode}', fallback to 'hold'")
            self.mode = 'hold'

        self.command_pub = self.create_publisher(Float64, '/control_command', 10)

        self.start_time_sec = self.get_clock().now().nanoseconds * 1e-9
        self.last_log_sec = 0.0
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.timer_callback)

        self.get_logger().info(
            f'AK70 command test node started | mode={self.mode}, rate={self.publish_rate_hz:.1f}Hz'
        )

    def compute_target_rad(self, elapsed_sec: float) -> float:
        if self.mode == 'hold':
            return self.hold_target_rad

        if self.mode == 'step':
            # 0.0 -> target -> 0.0 패턴을 동일 시간으로 반복한다.
            phase = elapsed_sec % (3.0 * self.step_hold_sec)
            if phase < self.step_hold_sec:
                return 0.0
            if phase < 2.0 * self.step_hold_sec:
                return self.step_target_rad
            return 0.0

        # sine 모드는 offset + amplitude * sin(2*pi*f*t) 형태를 사용한다.
        return self.sine_offset_rad + self.sine_amplitude_rad * math.sin(
            2.0 * math.pi * self.sine_frequency_hz * elapsed_sec
        )

    def timer_callback(self) -> None:
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        elapsed_sec = now_sec - self.start_time_sec
        target_rad = self.compute_target_rad(elapsed_sec)

        msg = Float64()
        msg.data = float(target_rad)
        self.command_pub.publish(msg)

        if now_sec - self.last_log_sec >= 1.0:
            self.get_logger().info(f'mode={self.mode} | target={target_rad:+.3f} rad')
            self.last_log_sec = now_sec


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AK70CommandTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
