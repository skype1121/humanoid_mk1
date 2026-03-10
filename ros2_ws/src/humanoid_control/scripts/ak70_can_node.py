#!/usr/bin/env python3

import queue
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float64

from humanoid_control_py.ak70_driver_core import AK70BusCollection
from humanoid_control_py.ak70_driver_core import AK70Motor
from humanoid_control_py.ak70_driver_core import AK70MotorConfig
from humanoid_control_py.ak70_driver_core import P_MAX
from humanoid_control_py.ak70_driver_core import P_MIN
from humanoid_control_py.ak70_driver_core import pack_mit_command
from humanoid_control_py.ak70_driver_core import parse_feedback_frame


class AK70CanNode(Node):
    def __init__(self) -> None:
        super().__init__('ak70_can_node')

        self.declare_parameter('can_interface', 'can0')
        # 기존 파라미터 호환을 위해 유지한다.
        self.declare_parameter('interface', '')
        self.declare_parameter('motor_id', 0x05)
        self.declare_parameter('control_period_sec', 0.01)
        # 기존 파라미터 호환을 위해 유지한다.
        self.declare_parameter('control_dt', 0.01)
        self.declare_parameter('kp', 12.0)
        self.declare_parameter('kd', 1.0)
        self.declare_parameter('command_timeout_sec', 0.5)
        self.declare_parameter('enable_motor_on_startup', False)
        # 가정 기반 구현:
        # startup enable이 꺼져 있어도 첫 제어 명령 시 자동 enable이 필요할 수 있어 옵션으로 둔다.
        self.declare_parameter('auto_enable_on_first_command', True)
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
        self.auto_enable_on_first_command = bool(self.get_parameter('auto_enable_on_first_command').value)
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

        command_qos = QoSProfile(depth=10)
        self.command_sub = self.create_subscription(
            Float64,
            '/control_command',
            self.control_command_callback,
            command_qos,
        )
        self.state_pub = self.create_publisher(Float64, '/motor_state', 10)
        self.command_queue = queue.Queue(maxsize=1)
        self.state_lock = threading.Lock()
        self.shutdown_event = threading.Event()
        self.latest_state_position = 0.0
        self.last_control_loop_log_monotonic = 0.0
        self.last_control_loop_cycle_start = None
        self.last_send_position_log_monotonic = 0.0
        self.last_feedback_log_monotonic = 0.0
        self.last_command_debug_log_monotonic = 0.0
        self.motor_mode_enabled = False
        self.last_raw_command_value = 0.0
        self.last_queue_command_value = 0.0
        self.last_joint_command_before_clamp = 0.0
        self.last_joint_command_after_clamp = 0.0
        self.last_motor_command_value = 0.0

        if self.send_zero_on_startup and not self.enable_motor_on_startup:
            self.get_logger().warn('send_zero_on_startup=true but enable_motor_on_startup=false; zero commands may be ignored')

        if self.enable_motor_on_startup:
            self.enter_motor_mode()
            self.motor_mode_enabled = True
            time.sleep(0.1)
        if self.send_zero_on_startup and self.startup_zero_command_repeat > 0:
            self.zero_command(
                repeat=self.startup_zero_command_repeat,
                dt=self.startup_zero_command_dt,
            )

        self.state_publish_timer = self.create_timer(self.control_period_sec, self.publish_motor_state)
        self.control_thread = threading.Thread(
            target=self.control_loop,
            name='ak70_control_thread',
            daemon=True,
        )
        self.control_thread.start()
        self.get_logger().info(
            f'AK70 CAN driver started | interface={self.interface}, motor_id=0x{self.motor_id:02X}, '
            f'enable_on_startup={self.enable_motor_on_startup}, auto_enable_on_first_command={self.auto_enable_on_first_command}, '
            f'zero_on_startup={self.send_zero_on_startup}'
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

    def clamp_motor_position(self, motor_position: float) -> float:
        # MIT 프로토콜 위치 범위를 넘는 명령은 패킹 전에 명시적으로 clamp해 로그로 추적한다.
        return max(P_MIN, min(P_MAX, motor_position))

    def send_position(self, target_pos: float, kp: float = 12.0, kd: float = 1.0) -> None:
        joint_command_before_clamp = float(target_pos)
        joint_command_after_clamp = joint_command_before_clamp
        motor_position_before_clamp = self.motor.command_to_motor_position(joint_command_after_clamp)
        motor_position = self.clamp_motor_position(motor_position_before_clamp)

        with self.state_lock:
            self.last_joint_command_before_clamp = joint_command_before_clamp
            self.last_joint_command_after_clamp = joint_command_after_clamp
            self.last_motor_command_value = motor_position

        data = pack_mit_command(
            p=motor_position,
            v=0.0,
            kp=kp,
            kd=kd,
            t_ff=0.0,
        )
        now_sec = time.monotonic()
        # 제어 명령 송신 로그는 2초마다 한 번만 출력해 실제 송신 여부만 확인한다.
        if now_sec - self.last_send_position_log_monotonic >= 2.0:
            self.last_send_position_log_monotonic = now_sec
            clamp_note = ''
            if abs(motor_position - motor_position_before_clamp) > 1e-9:
                clamp_note = f', motor_clamped_from={motor_position_before_clamp:.3f}'
            self.get_logger().info(
                f'send_position called: target_joint={joint_command_after_clamp:.3f} rad, '
                f'target_motor={motor_position:.3f} rad{clamp_note}, kp={kp:.2f}, kd={kd:.2f}, '
                f'motor_mode_enabled={self.motor_mode_enabled}'
            )
        self.send_frame(data)

    def read_feedback(self, timeout: float = 0.02):
        msg = self.bus_collection.recv(self.interface, timeout=timeout)
        if msg is None:
            return None
        if msg.arbitration_id != self.motor_id:
            return None
        feedback = parse_feedback_frame(msg)
        if feedback is None:
            self.get_logger().warn('Received feedback frame but failed to parse it')
            return None

        now_sec = time.monotonic()
        # 피드백 파싱 로그는 2초마다 한 번만 출력해 정상 수신 여부를 확인한다.
        if now_sec - self.last_feedback_log_monotonic >= 2.0:
            self.last_feedback_log_monotonic = now_sec
            self.get_logger().info(
                f'feedback parsed: id={int(feedback["id"]):d}, pos={float(feedback["pos"]):.3f}, '
                f'vel={float(feedback["vel"]):.3f}, torque={float(feedback["torque"]):.3f}'
            )
        return feedback

    def control_command_callback(self, msg: Float64) -> None:
        now_sec = time.monotonic()
        target_position = float(msg.data)
        latest_command = (target_position, now_sec)

        self.get_logger().info(f'callback received: {msg.data}')

        with self.state_lock:
            self.last_raw_command_value = target_position
            # callback에서 받은 값을 그대로 내부 목표값으로 기록해 전달 경로를 직접 확인한다.
            self.motor.target_position = target_position

        # 가장 최신 명령만 유지하기 위해 기존 항목이 있으면 버리고 새 항목으로 교체한다.
        if self.command_queue.full():
            try:
                self.command_queue.get_nowait()
            except queue.Empty:
                pass

        try:
            self.command_queue.put_nowait(latest_command)
            with self.state_lock:
                self.last_queue_command_value = target_position
        except queue.Full:
            pass

    def update_latest_command(self) -> None:
        while True:
            try:
                target_position, command_time = self.command_queue.get_nowait()
            except queue.Empty:
                return
            with self.state_lock:
                self.motor.target_position = target_position
                self.motor.last_command_monotonic = command_time

            if self.auto_enable_on_first_command and not self.motor_mode_enabled:
                self.get_logger().info('First control command received; entering motor mode automatically')
                self.enter_motor_mode()
                self.motor_mode_enabled = True
                time.sleep(0.1)

    def control_loop(self) -> None:
        next_cycle_time = time.monotonic()
        while not self.shutdown_event.is_set():
            cycle_start = time.monotonic()
            self.update_latest_command()
            self.run_control_cycle()
            self.log_control_loop_status(cycle_start)
            next_cycle_time += self.control_period_sec
            sleep_sec = next_cycle_time - time.monotonic()
            if sleep_sec > 0.0:
                self.shutdown_event.wait(sleep_sec)
            else:
                next_cycle_time = time.monotonic()

    def log_control_loop_status(self, cycle_start: float) -> None:
        if self.last_control_loop_cycle_start is None:
            actual_loop_hz = 0.0
        else:
            cycle_dt = cycle_start - self.last_control_loop_cycle_start
            actual_loop_hz = 0.0 if cycle_dt <= 0.0 else 1.0 / cycle_dt

        self.last_control_loop_cycle_start = cycle_start

        # 제어 루프 상태 로그는 5초마다 한 번만 출력해 로그 과밀을 방지한다.
        if cycle_start - self.last_control_loop_log_monotonic < 5.0:
            return

        self.last_control_loop_log_monotonic = cycle_start
        configured_loop_hz = 1.0 / self.control_period_sec if self.control_period_sec > 0.0 else 0.0
        self.get_logger().info(
            f'control loop running: configured={configured_loop_hz:.1f}Hz, actual={actual_loop_hz:.1f}Hz, '
            f'feedback_timeout={self.feedback_timeout_sec:.4f}s'
        )

    def run_control_cycle(self) -> None:
        with self.state_lock:
            raw_command_value = self.last_raw_command_value
            queue_command_value = self.last_queue_command_value
            last_command_monotonic = self.motor.last_command_monotonic
            target_position = self.motor.target_position
            timeout_warn_monotonic = self.motor.last_timeout_warn_monotonic

        now_sec = time.monotonic()
        # 명령 전달 경로를 확인하기 위해 raw 입력값과 실제 제어값을 함께 출력한다.
        if now_sec - self.last_command_debug_log_monotonic >= 2.0:
            self.last_command_debug_log_monotonic = now_sec
            self.get_logger().info(
                f'command trace: raw={raw_command_value:.3f}, queued={queue_command_value:.3f}, '
                f'control_target={target_position:.3f}'
            )

        if last_command_monotonic is not None:
            elapsed_sec = now_sec - last_command_monotonic
            if elapsed_sec > self.command_timeout_sec:
                # 명령 타임아웃 시 마지막 목표를 유지한다.
                if now_sec - timeout_warn_monotonic > 1.0:
                    self.get_logger().warn(
                        f'control_command timeout ({elapsed_sec:.2f}s): holding last target {target_position:.3f} rad'
                    )
                    with self.state_lock:
                        self.motor.last_timeout_warn_monotonic = now_sec

        self.send_position(target_position, kp=self.kp, kd=self.kd)
        feedback = self.read_feedback(timeout=self.feedback_timeout_sec)

        if feedback is None:
            now_sec = time.monotonic()
            with self.state_lock:
                last_feedback_warn_monotonic = self.motor.last_feedback_warn_monotonic
            if now_sec - last_feedback_warn_monotonic > 2.0:
                self.get_logger().warn('No motor feedback received (throttled)')
                with self.state_lock:
                    self.motor.last_feedback_warn_monotonic = now_sec
            return

        with self.state_lock:
            self.motor.feedback_to_joint_state(feedback)
            self.latest_state_position = float(self.motor.state.position)

    def publish_motor_state(self) -> None:
        with self.state_lock:
            position = self.latest_state_position
        msg = Float64()
        msg.data = position
        self.state_pub.publish(msg)

    def shutdown_driver(self) -> None:
        self.get_logger().info('Shutting down AK70 CAN driver')
        try:
            self.shutdown_event.set()
            if hasattr(self, 'control_thread') and self.control_thread.is_alive():
                self.control_thread.join(timeout=1.0)
            if self.motor_mode_enabled:
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
