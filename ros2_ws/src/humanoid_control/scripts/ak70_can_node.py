#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from humanoid_control_py.ak70_driver_core import AK70BusCollection
from humanoid_control_py.ak70_driver_core import AK70Motor
from humanoid_control_py.ak70_driver_core import AK70MotorConfig
from humanoid_control_py.ak70_driver_core import pack_mit_command
from humanoid_control_py.ak70_driver_core import parse_feedback_frame


class AK70CanNode(Node):
    def __init__(self) -> None:
        super().__init__('ak70_can_node')

        self.declare_parameter('can_interface', 'can0')
        # 기존 파라미터 호환을 위해 유지한다.
        self.declare_parameter('interface', '')
        self.declare_parameter('motor_id', 0x05)
        self.declare_parameter('control_period_sec', 0.02)
        # 기존 파라미터 호환을 위해 유지한다.
        self.declare_parameter('control_dt', 0.02)
        self.declare_parameter('kp', 12.0)
        self.declare_parameter('kd', 1.0)
        self.declare_parameter('command_timeout_sec', 0.5)
        self.declare_parameter('enable_motor_on_startup', False)
        self.declare_parameter('send_zero_on_startup', False)
        self.declare_parameter('startup_zero_command_repeat', 50)
        self.declare_parameter('startup_zero_command_dt', 0.01)
        self.declare_parameter('feedback_timeout_sec', 0.002)

        can_interface = str(self.get_parameter('can_interface').value)
        legacy_interface = str(self.get_parameter('interface').value)
        self.interface = legacy_interface if legacy_interface else can_interface
        self.motor_id = int(self.get_parameter('motor_id').value)
        self.control_period_sec = float(self.get_parameter('control_period_sec').value)
        if self.control_period_sec <= 0.0:
            self.control_period_sec = float(self.get_parameter('control_dt').value)
        if self.control_period_sec <= 0.0:
            self.control_period_sec = 0.02

        self.kp = float(self.get_parameter('kp').value)
        self.kd = float(self.get_parameter('kd').value)
        self.command_timeout_sec = float(self.get_parameter('command_timeout_sec').value)
        self.enable_motor_on_startup = bool(self.get_parameter('enable_motor_on_startup').value)
        self.send_zero_on_startup = bool(self.get_parameter('send_zero_on_startup').value)
        self.startup_zero_command_repeat = int(self.get_parameter('startup_zero_command_repeat').value)
        self.startup_zero_command_dt = float(self.get_parameter('startup_zero_command_dt').value)
        self.feedback_timeout_sec = float(self.get_parameter('feedback_timeout_sec').value)

        self.bus_collection = AK70BusCollection([self.interface])
        self.motor = AK70Motor(
            AK70MotorConfig(
                joint_name='ak70_test_joint',
                motor_id=self.motor_id,
                can_interface=self.interface,
                direction=1.0,
                zero_offset=0.0,
                kp=self.kp,
                kd=self.kd,
            )
        )

        self.command_sub = self.create_subscription(
            Float64,
            '/control_command',
            self.control_command_callback,
            10,
        )
        self.state_pub = self.create_publisher(Float64, '/motor_state', 10)

        if self.send_zero_on_startup and not self.enable_motor_on_startup:
            self.get_logger().warn('send_zero_on_startup=true but enable_motor_on_startup=false; zero commands may be ignored')

        if self.enable_motor_on_startup:
            self.enter_motor_mode()
            time.sleep(0.1)
        if self.send_zero_on_startup and self.startup_zero_command_repeat > 0:
            self.zero_command(
                repeat=self.startup_zero_command_repeat,
                dt=self.startup_zero_command_dt,
            )

        self.control_timer = self.create_timer(self.control_period_sec, self.control_timer_callback)
        self.get_logger().info(
            f'AK70 CAN driver started | interface={self.interface}, motor_id=0x{self.motor_id:02X}, '
            f'enable_on_startup={self.enable_motor_on_startup}, zero_on_startup={self.send_zero_on_startup}'
        )

    def send_frame(self, data: bytes) -> None:
        self.bus_collection.send(self.interface, self.motor_id, data)

    def enter_motor_mode(self) -> None:
        self.send_frame(bytes.fromhex('FF FF FF FF FF FF FF FC'))

    def exit_motor_mode(self) -> None:
        self.send_frame(bytes.fromhex('FF FF FF FF FF FF FF FD'))

    def zero_command(self, repeat: int = 50, dt: float = 0.01) -> None:
        data = pack_mit_command(
            p=0.0,
            v=0.0,
            kp=0.0,
            kd=0.0,
            t_ff=0.0,
        )
        for _ in range(repeat):
            self.send_frame(data)
            time.sleep(dt)

    def send_position(self, target_pos: float, kp: float = 12.0, kd: float = 1.0) -> None:
        motor_position = self.motor.command_to_motor_position(target_pos)
        data = pack_mit_command(
            p=motor_position,
            v=0.0,
            kp=kp,
            kd=kd,
            t_ff=0.0,
        )
        self.send_frame(data)

    def read_feedback(self, timeout: float = 0.02):
        msg = self.bus_collection.recv(self.interface, timeout=timeout)
        if msg is None:
            return None
        if msg.arbitration_id != self.motor_id:
            return None
        return parse_feedback_frame(msg)

    def control_command_callback(self, msg: Float64) -> None:
        self.motor.target_position = float(msg.data)
        self.motor.last_command_monotonic = time.monotonic()

    def control_timer_callback(self) -> None:
        now_sec = time.monotonic()

        if self.motor.last_command_monotonic is not None:
            elapsed_sec = now_sec - self.motor.last_command_monotonic
            if elapsed_sec > self.command_timeout_sec:
                # 명령 타임아웃 시 마지막 목표를 유지한다.
                if now_sec - self.motor.last_timeout_warn_monotonic > 1.0:
                    self.get_logger().warn(
                        f'control_command timeout ({elapsed_sec:.2f}s): holding last target {self.motor.target_position:.3f} rad'
                    )
                    self.motor.last_timeout_warn_monotonic = now_sec

        self.send_position(self.motor.target_position, kp=self.kp, kd=self.kd)
        feedback = self.read_feedback(timeout=self.feedback_timeout_sec)

        if feedback is None:
            if now_sec - self.motor.last_feedback_warn_monotonic > 2.0:
                self.get_logger().warn('No motor feedback received (throttled)')
                self.motor.last_feedback_warn_monotonic = now_sec
            return

        self.motor.feedback_to_joint_state(feedback)

        msg = Float64()
        msg.data = float(self.motor.state.position)
        self.state_pub.publish(msg)

    def shutdown_driver(self) -> None:
        self.get_logger().info('Shutting down AK70 CAN driver')
        try:
            if self.enable_motor_on_startup:
                self.exit_motor_mode()
                time.sleep(0.1)
        finally:
            self.bus_collection.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AK70CanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_driver()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
