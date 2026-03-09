#!/usr/bin/env python3

import time

import can
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

# MIT 프로토콜 공통 범위
P_MIN = -12.5
P_MAX = 12.5
V_MIN = -65.0
V_MAX = 65.0
KP_MIN = 0.0
KP_MAX = 500.0
KD_MIN = 0.0
KD_MAX = 5.0
T_MIN = -18.0
T_MAX = 18.0


class AK70CanNode(Node):
    def __init__(self) -> None:
        super().__init__('ak70_can_node')

        self.declare_parameter('can_interface', 'can0')
        # 기존 파라미터 호환을 위해 유지 (assumption-based).
        self.declare_parameter('interface', '')
        self.declare_parameter('motor_id', 0x05)
        self.declare_parameter('control_period_sec', 0.02)
        # 기존 파라미터 호환을 위해 유지 (assumption-based).
        self.declare_parameter('control_dt', 0.02)
        self.declare_parameter('kp', 12.0)
        self.declare_parameter('kd', 1.0)
        self.declare_parameter('command_timeout_sec', 0.5)
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
        self.startup_zero_command_repeat = int(self.get_parameter('startup_zero_command_repeat').value)
        self.startup_zero_command_dt = float(self.get_parameter('startup_zero_command_dt').value)
        self.feedback_timeout_sec = float(self.get_parameter('feedback_timeout_sec').value)

        self.bus = can.Bus(interface='socketcan', channel=self.interface)

        self.target_position = 0.0
        self.last_command_time = None
        self.last_feedback_warn_time = 0.0
        self.last_timeout_warn_time = 0.0

        self.command_sub = self.create_subscription(
            Float64,
            '/control_command',
            self.control_command_callback,
            10,
        )
        self.state_pub = self.create_publisher(Float64, '/motor_state', 10)

        self.enter_motor_mode()
        time.sleep(0.1)
        if self.startup_zero_command_repeat > 0:
            self.zero_command(
                repeat=self.startup_zero_command_repeat,
                dt=self.startup_zero_command_dt,
            )

        self.control_timer = self.create_timer(self.control_period_sec, self.control_timer_callback)
        self.get_logger().info(
            f'AK70 CAN driver started | interface={self.interface}, motor_id=0x{self.motor_id:02X}'
        )

    def float_to_uint(self, x: float, x_min: float, x_max: float, bits: int) -> int:
        span = x_max - x_min
        offset = x - x_min
        max_int = (1 << bits) - 1
        value = int(offset * max_int / span)

        if value < 0:
            return 0
        if value > max_int:
            return max_int
        return value

    def uint_to_float(self, x: int, x_min: float, x_max: float, bits: int) -> float:
        span = x_max - x_min
        max_int = (1 << bits) - 1
        return float(x) * span / max_int + x_min

    def pack_mit_command(self, p: float, v: float, kp: float, kd: float, t_ff: float) -> bytes:
        p_int = self.float_to_uint(p, P_MIN, P_MAX, 16)
        v_int = self.float_to_uint(v, V_MIN, V_MAX, 12)
        kp_int = self.float_to_uint(kp, KP_MIN, KP_MAX, 12)
        kd_int = self.float_to_uint(kd, KD_MIN, KD_MAX, 12)
        t_int = self.float_to_uint(t_ff, T_MIN, T_MAX, 12)

        data = bytearray(8)
        data[0] = (p_int >> 8) & 0xFF
        data[1] = p_int & 0xFF
        data[2] = (v_int >> 4) & 0xFF
        data[3] = ((v_int & 0x0F) << 4) | ((kp_int >> 8) & 0x0F)
        data[4] = kp_int & 0xFF
        data[5] = (kd_int >> 4) & 0xFF
        data[6] = ((kd_int & 0x0F) << 4) | ((t_int >> 8) & 0x0F)
        data[7] = t_int & 0xFF
        return bytes(data)

    def send_frame(self, data: bytes) -> None:
        msg = can.Message(
            arbitration_id=self.motor_id,
            data=data,
            is_extended_id=False,
        )
        self.bus.send(msg)

    def enter_motor_mode(self) -> None:
        self.send_frame(bytes.fromhex('FF FF FF FF FF FF FF FC'))

    def exit_motor_mode(self) -> None:
        self.send_frame(bytes.fromhex('FF FF FF FF FF FF FF FD'))

    def zero_command(self, repeat: int = 50, dt: float = 0.01) -> None:
        data = self.pack_mit_command(
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
        data = self.pack_mit_command(
            p=target_pos,
            v=0.0,
            kp=kp,
            kd=kd,
            t_ff=0.0,
        )
        self.send_frame(data)

    def parse_feedback(self, msg: can.Message):
        # MIT 피드백 프레임을 위치/속도/토크로 디코딩한다.
        if len(msg.data) < 6:
            return None

        data = msg.data

        p_int = (data[1] << 8) | data[2]
        v_int = (data[3] << 4) | (data[4] >> 4)
        t_int = ((data[4] & 0x0F) << 8) | data[5]

        return {
            'id': data[0],
            'pos': self.uint_to_float(p_int, P_MIN, P_MAX, 16),
            'vel': self.uint_to_float(v_int, V_MIN, V_MAX, 12),
            'torque': self.uint_to_float(t_int, T_MIN, T_MAX, 12),
        }

    def read_feedback(self, timeout: float = 0.02):
        msg = self.bus.recv(timeout=timeout)
        if msg is None:
            return None
        if msg.arbitration_id != self.motor_id:
            return None
        return self.parse_feedback(msg)

    def control_command_callback(self, msg: Float64) -> None:
        self.target_position = float(msg.data)
        self.last_command_time = self.get_clock().now()

    def control_timer_callback(self) -> None:
        now = self.get_clock().now()
        now_sec = now.nanoseconds * 1e-9

        if self.last_command_time is not None:
            elapsed_sec = (now - self.last_command_time).nanoseconds * 1e-9
            if elapsed_sec > self.command_timeout_sec:
                # 명령 타임아웃 시 마지막 목표를 유지한다.
                if now_sec - self.last_timeout_warn_time > 1.0:
                    self.get_logger().warn(
                        f'control_command timeout ({elapsed_sec:.2f}s): holding last target {self.target_position:.3f} rad'
                    )
                    self.last_timeout_warn_time = now_sec

        self.send_position(self.target_position, kp=self.kp, kd=self.kd)
        feedback = self.read_feedback(timeout=self.feedback_timeout_sec)

        if feedback is None:
            if now_sec - self.last_feedback_warn_time > 2.0:
                self.get_logger().warn('No motor feedback received (throttled)')
                self.last_feedback_warn_time = now_sec
            return

        msg = Float64()
        msg.data = float(feedback['pos'])
        self.state_pub.publish(msg)

    def shutdown_driver(self) -> None:
        self.get_logger().info('Shutting down AK70 CAN driver')
        try:
            self.exit_motor_mode()
            time.sleep(0.1)
        finally:
            self.bus.shutdown()


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
