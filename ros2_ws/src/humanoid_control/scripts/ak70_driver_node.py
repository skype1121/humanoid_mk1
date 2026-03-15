#!/usr/bin/env python3

import time
from typing import Dict, List, Optional

import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from humanoid_control.msg import DriverCommand
from humanoid_control.msg import MotorCommand
from humanoid_control.msg import MotorCommandArray
from humanoid_control.msg import MotorState
from humanoid_control.msg import MotorStateArray

from humanoid_control_py.ak70_driver_core import AK70BusCollection
from humanoid_control_py.ak70_driver_core import AK70Motor
from humanoid_control_py.ak70_driver_core import AK70MotorConfig
from humanoid_control_py.ak70_driver_core import P_MAX
from humanoid_control_py.ak70_driver_core import P_MIN
from humanoid_control_py.ak70_driver_core import pack_mit_command
from humanoid_control_py.ak70_driver_core import parse_feedback_frame


class AK70DriverNode(Node):
    def __init__(self) -> None:
        super().__init__('ak70_driver_node')

        self.declare_parameter('control_period_sec', 0.02)
        self.declare_parameter('command_timeout_sec', 0.5)
        self.declare_parameter('feedback_timeout_sec', 0.002)
        self.declare_parameter('enable_motors_on_startup', False)
        self.declare_parameter('send_zero_on_startup', False)
        self.declare_parameter('startup_zero_command_repeat', 50)
        self.declare_parameter('startup_zero_command_dt', 0.01)
        self.declare_parameter('command_topic', '/ak70/command')
        self.declare_parameter('state_topic', '/ak70/motor_states')
        self.declare_parameter('motor_command_topic', '/ak70/motor_commands')
        self.declare_parameter('motor_state_topic', '/ak70/motor_states_detailed')
        self.declare_parameter('driver_command_topic', '/ak70/driver_command')
        self.declare_parameter('legacy_command_topic', '/control_command')
        self.declare_parameter('legacy_state_topic', '/motor_state')
        self.declare_parameter('enable_legacy_topics', True)
        self.declare_parameter('legacy_joint_name', '')
        self.declare_parameter('joint_names', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('motor_ids', Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter('can_interfaces', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('directions', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('zero_offsets', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('kps', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('kds', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('min_positions', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('max_positions', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('max_position_steps', Parameter.Type.DOUBLE_ARRAY)

        self.control_period_sec = float(self.get_parameter('control_period_sec').value)
        if self.control_period_sec <= 0.0:
            self.control_period_sec = 0.02
        self.command_timeout_sec = float(self.get_parameter('command_timeout_sec').value)
        self.feedback_timeout_sec = float(self.get_parameter('feedback_timeout_sec').value)
        self.enable_motors_on_startup = bool(self.get_parameter('enable_motors_on_startup').value)
        self.send_zero_on_startup = bool(self.get_parameter('send_zero_on_startup').value)
        self.startup_zero_command_repeat = int(self.get_parameter('startup_zero_command_repeat').value)
        self.startup_zero_command_dt = float(self.get_parameter('startup_zero_command_dt').value)
        self.command_topic = str(self.get_parameter('command_topic').value)
        self.state_topic = str(self.get_parameter('state_topic').value)
        self.motor_command_topic = str(self.get_parameter('motor_command_topic').value)
        self.motor_state_topic = str(self.get_parameter('motor_state_topic').value)
        self.driver_command_topic = str(self.get_parameter('driver_command_topic').value)
        self.legacy_command_topic = str(self.get_parameter('legacy_command_topic').value)
        self.legacy_state_topic = str(self.get_parameter('legacy_state_topic').value)
        self.enable_legacy_topics = bool(self.get_parameter('enable_legacy_topics').value)
        self.legacy_joint_name = str(self.get_parameter('legacy_joint_name').value).strip()

        self.motors = self.load_motor_configs()
        self.bus_collection = AK70BusCollection(
            [motor.config.can_interface for motor in self.motors]
        )
        self.motor_by_joint_name: Dict[str, AK70Motor] = {
            motor.config.joint_name: motor for motor in self.motors
        }
        self.motor_by_id: Dict[int, AK70Motor] = {
            motor.config.motor_id: motor for motor in self.motors
        }
        self.motors_by_interface: Dict[str, List[AK70Motor]] = {}
        for motor in self.motors:
            self.motors_by_interface.setdefault(motor.config.can_interface, []).append(motor)

        self.command_sub = self.create_subscription(
            JointState,
            self.command_topic,
            self.joint_state_command_callback,
            10,
        )
        self.motor_command_sub = self.create_subscription(
            MotorCommandArray,
            self.motor_command_topic,
            self.motor_command_callback,
            10,
        )
        self.driver_command_sub = self.create_subscription(
            DriverCommand,
            self.driver_command_topic,
            self.driver_command_callback,
            10,
        )
        self.state_pub = self.create_publisher(JointState, self.state_topic, 10)
        self.motor_state_pub = self.create_publisher(MotorStateArray, self.motor_state_topic, 10)

        self.legacy_motor = self.resolve_legacy_motor()
        self.legacy_command_sub = None
        self.legacy_state_pub = None
        if self.enable_legacy_topics and self.legacy_motor is not None:
            self.legacy_command_sub = self.create_subscription(
                Float64,
                self.legacy_command_topic,
                self.legacy_command_callback,
                10,
            )
            self.legacy_state_pub = self.create_publisher(Float64, self.legacy_state_topic, 10)

        if self.send_zero_on_startup and not self.enable_motors_on_startup:
            self.get_logger().warn('send_zero_on_startup=true but enable_motors_on_startup=false; zero commands may be ignored')

        if self.enable_motors_on_startup:
            self.enter_motor_mode_all()
            for motor in self.motors:
                motor.enabled = True
            time.sleep(0.1)
        if self.send_zero_on_startup and self.startup_zero_command_repeat > 0:
            self.zero_command_all(
                repeat=self.startup_zero_command_repeat,
                dt=self.startup_zero_command_dt,
            )

        self.control_timer = self.create_timer(self.control_period_sec, self.control_timer_callback)
        self.last_unexpected_feedback_log_monotonic = 0.0
        self.last_parse_failure_log_monotonic = 0.0
        self.last_clamp_log_by_joint = {motor.config.joint_name: 0.0 for motor in self.motors}
        self.last_step_limit_log_by_joint = {motor.config.joint_name: 0.0 for motor in self.motors}
        self.get_logger().info(
            f'AK70 multi driver started | motors={len(self.motors)}, interfaces={sorted(self.motors_by_interface.keys())}, '
            f'enable_on_startup={self.enable_motors_on_startup}, zero_on_startup={self.send_zero_on_startup}, '
            f'legacy_topics={self.enable_legacy_topics}'
        )
        self.get_logger().info(
            'Configured motors: '
            + ', '.join(
                f'{motor.config.joint_name}(id={motor.config.motor_id}, can={motor.config.can_interface})'
                for motor in self.motors
            )
        )

    def validate_required_length(
        self,
        name: str,
        values: List[float],
        required_length: int,
        allow_empty: bool = False,
    ) -> List[float]:
        # 빈 배열 허용 항목은 기본값 확장을 위해 그대로 반환한다.
        if allow_empty and not values:
            return values
        if len(values) != required_length:
            raise ValueError(f'{name} length mismatch: {len(values)} != {required_length}')
        return values

    def load_motor_configs(self) -> List[AK70Motor]:
        joint_names = [str(value) for value in self.get_parameter('joint_names').value]
        motor_ids = [int(value) for value in self.get_parameter('motor_ids').value]
        can_interfaces = [str(value) for value in self.get_parameter('can_interfaces').value]
        directions = [float(value) for value in self.get_parameter('directions').value]
        zero_offsets = [float(value) for value in self.get_parameter('zero_offsets').value]
        kps = [float(value) for value in self.get_parameter('kps').value]
        kds = [float(value) for value in self.get_parameter('kds').value]
        min_positions = [float(value) for value in self.get_parameter('min_positions').value]
        max_positions = [float(value) for value in self.get_parameter('max_positions').value]
        max_position_steps = [float(value) for value in self.get_parameter('max_position_steps').value]

        counts = {
            'joint_names': len(joint_names),
            'motor_ids': len(motor_ids),
            'can_interfaces': len(can_interfaces),
            'directions': len(directions),
            'zero_offsets': len(zero_offsets),
        }
        required_length = len(joint_names)
        if required_length == 0:
            raise ValueError('joint_names parameter is empty')

        for key, value in counts.items():
            if value != required_length:
                raise ValueError(f'Parameter length mismatch: {key}={value}, expected={required_length}')

        self.validate_required_length('kps', kps, required_length, allow_empty=True)
        self.validate_required_length('kds', kds, required_length, allow_empty=True)
        self.validate_required_length('min_positions', min_positions, required_length, allow_empty=True)
        self.validate_required_length('max_positions', max_positions, required_length, allow_empty=True)
        self.validate_required_length('max_position_steps', max_position_steps, required_length, allow_empty=True)

        if not kps:
            kps = [12.0] * required_length
        if not kds:
            kds = [1.0] * required_length
        if not min_positions:
            min_positions = [P_MIN] * required_length
        if not max_positions:
            max_positions = [P_MAX] * required_length
        if not max_position_steps:
            max_position_steps = [0.05] * required_length

        motors: List[AK70Motor] = []
        seen_joint_names = set()
        seen_motor_ids = set()
        for index, joint_name in enumerate(joint_names):
            motor_id = motor_ids[index]
            if joint_name in seen_joint_names:
                raise ValueError(f'Duplicate joint_name: {joint_name}')
            if motor_id in seen_motor_ids:
                raise ValueError(f'Duplicate motor_id: {motor_id}')
            seen_joint_names.add(joint_name)
            seen_motor_ids.add(motor_id)
            motors.append(
                AK70Motor(
                    AK70MotorConfig(
                        joint_name=joint_name,
                        motor_id=motor_id,
                        can_interface=can_interfaces[index],
                        direction=directions[index],
                        zero_offset=zero_offsets[index],
                        kp=kps[index],
                        kd=kds[index],
                        min_position=min_positions[index],
                        max_position=max_positions[index],
                        max_step_per_cycle=max_position_steps[index],
                    )
                )
            )
        return motors

    def resolve_legacy_motor(self) -> Optional[AK70Motor]:
        if not self.motors:
            return None

        if not self.legacy_joint_name:
            return self.motors[0]

        motor = self.motor_by_joint_name.get(self.legacy_joint_name)
        if motor is None:
            self.get_logger().warn(
                f'legacy_joint_name "{self.legacy_joint_name}" not found; falling back to {self.motors[0].config.joint_name}'
            )
            return self.motors[0]
        return motor

    def send_motor_frame(self, motor: AK70Motor, data: bytes) -> None:
        self.bus_collection.send(
            motor.config.can_interface,
            motor.config.motor_id,
            data,
        )

    def enter_motor_mode_all(self) -> None:
        data = bytes.fromhex('FF FF FF FF FF FF FF FC')
        for motor in self.motors:
            self.send_motor_frame(motor, data)
            motor.enabled = True

    def exit_motor_mode_all(self) -> None:
        data = bytes.fromhex('FF FF FF FF FF FF FF FD')
        for motor in self.motors:
            self.send_motor_frame(motor, data)
            motor.enabled = False

    def zero_command_all(self, repeat: int, dt: float) -> None:
        data = pack_mit_command(
            p=0.0,
            v=0.0,
            kp=0.0,
            kd=0.0,
            t_ff=0.0,
        )
        for _ in range(repeat):
            for motor in self.motors:
                self.send_motor_frame(motor, data)
            time.sleep(dt)

    def set_motor_target(self, motor: AK70Motor, target_position: float, timestamp_sec: float) -> None:
        requested_position = float(target_position)
        clamped_position = motor.clamp_joint_position(requested_position)
        if abs(clamped_position - requested_position) > 1e-9:
            now_sec = time.monotonic()
            last_log_sec = self.last_clamp_log_by_joint[motor.config.joint_name]
            if now_sec - last_log_sec >= 1.0:
                self.get_logger().warn(
                    f'Command clamp applied for {motor.config.joint_name}: '
                    f'requested={requested_position:.3f} rad, applied={clamped_position:.3f} rad, '
                    f'limit=[{motor.config.min_position:.3f}, {motor.config.max_position:.3f}]'
                )
                self.last_clamp_log_by_joint[motor.config.joint_name] = now_sec

        motor.target_position = clamped_position
        motor.last_command_monotonic = timestamp_sec

    def joint_state_command_callback(self, msg: JointState) -> None:
        if not msg.name:
            self.get_logger().warn('Received JointState command without joint names')
            return

        if len(msg.position) < len(msg.name):
            self.get_logger().warn('Received JointState command with insufficient position entries')
            return

        now_sec = time.monotonic()
        # 일부 joint만 들어오면 해당 joint만 target을 갱신하고,
        # 나머지 joint는 마지막 target을 그대로 유지한다.
        for index, joint_name in enumerate(msg.name):
            motor = self.motor_by_joint_name.get(joint_name)
            if motor is None:
                self.get_logger().warn(f'Ignoring command for unknown joint "{joint_name}"')
                continue
            self.set_motor_target(motor, float(msg.position[index]), now_sec)

    def motor_command_callback(self, msg: MotorCommandArray) -> None:
        now_sec = time.monotonic()
        for command in msg.commands:
            motor = None
            if command.joint_name:
                motor = self.motor_by_joint_name.get(command.joint_name)
            elif int(command.motor_id) in self.motor_by_id:
                motor = self.motor_by_id[int(command.motor_id)]

            if motor is None:
                self.get_logger().warn(
                    f'Ignoring custom command: unknown joint="{command.joint_name}" motor_id={int(command.motor_id)}'
                )
                continue

            self.set_motor_target(motor, float(command.target_position), now_sec)
            if command.has_gains:
                motor.config.kp = float(command.kp)
                motor.config.kd = float(command.kd)

    def legacy_command_callback(self, msg: Float64) -> None:
        if self.legacy_motor is None:
            return
        self.set_motor_target(self.legacy_motor, float(msg.data), time.monotonic())

    def stop_all_motors(self) -> None:
        now_sec = time.monotonic()
        for motor in self.motors:
            hold_position = motor.state.position
            motor.target_position = motor.clamp_joint_position(hold_position)
            motor.last_command_monotonic = now_sec

    def apply_zero_pose(self, joint_names: List[str], target_positions: List[float]) -> None:
        now_sec = time.monotonic()
        if joint_names and len(joint_names) == len(target_positions):
            for index, joint_name in enumerate(joint_names):
                motor = self.motor_by_joint_name.get(joint_name)
                if motor is None:
                    self.get_logger().warn(f'Ignoring zero pose target for unknown joint "{joint_name}"')
                    continue
                self.set_motor_target(motor, float(target_positions[index]), now_sec)
            return

        for motor in self.motors:
            self.set_motor_target(motor, 0.0, now_sec)

    def driver_command_callback(self, msg: DriverCommand) -> None:
        command = str(msg.command).strip().upper()
        if not command:
            return

        if command == 'ENABLE_ALL':
            self.enter_motor_mode_all()
            return
        if command == 'DISABLE_ALL':
            self.exit_motor_mode_all()
            return
        if command == 'ZERO_POSE':
            self.apply_zero_pose(list(msg.joint_names), list(msg.target_positions))
            return
        if command == 'STOP':
            self.stop_all_motors()
            return

        self.get_logger().warn(f'Unsupported driver command "{msg.command}"')

    def send_targets(self) -> None:
        now_sec = time.monotonic()
        for motor in self.motors:
            if not motor.enabled:
                continue

            if motor.last_command_monotonic is not None:
                elapsed_sec = now_sec - motor.last_command_monotonic
                if elapsed_sec > self.command_timeout_sec:
                    if now_sec - motor.last_timeout_warn_monotonic > 1.0:
                        self.get_logger().warn(
                            f'command timeout for {motor.config.joint_name} ({elapsed_sec:.2f}s): holding last target {motor.target_position:.3f} rad'
                        )
                        motor.last_timeout_warn_monotonic = now_sec

            limited_target = motor.limit_target_step(motor.target_position)
            if abs(limited_target - motor.target_position) > 1e-9:
                last_log_sec = self.last_step_limit_log_by_joint[motor.config.joint_name]
                if now_sec - last_log_sec >= 1.0:
                    self.get_logger().warn(
                        f'Command step limited for {motor.config.joint_name}: '
                        f'target={motor.target_position:.3f} rad, sent={limited_target:.3f} rad, '
                        f'max_step={motor.config.max_step_per_cycle:.3f} rad/cycle'
                    )
                    self.last_step_limit_log_by_joint[motor.config.joint_name] = now_sec
            motor.last_sent_target_position = limited_target
            motor_position = motor.command_to_motor_position(limited_target)
            data = pack_mit_command(
                p=motor_position,
                v=0.0,
                kp=motor.config.kp,
                kd=motor.config.kd,
                t_ff=0.0,
            )
            self.send_motor_frame(motor, data)

    def poll_feedback(self) -> None:
        now_sec = time.monotonic()
        for interface_name, interface_motors in self.motors_by_interface.items():
            active_motors = [motor for motor in interface_motors if motor.enabled]
            if not active_motors:
                continue

            expected_ids = {motor.config.motor_id for motor in active_motors}
            pending_ids = set(expected_ids)
            deadline = time.monotonic() + max(self.feedback_timeout_sec, 0.0)

            while pending_ids:
                remaining_timeout = deadline - time.monotonic()
                if remaining_timeout <= 0.0:
                    break

                feedback_msg = self.bus_collection.recv(interface_name, timeout=remaining_timeout)
                if feedback_msg is None:
                    break

                motor = self.motor_by_id.get(feedback_msg.arbitration_id)
                if motor is None:
                    if now_sec - self.last_unexpected_feedback_log_monotonic >= 1.0:
                        self.get_logger().warn(
                            f'Unexpected feedback id on {interface_name}: '
                            f'0x{feedback_msg.arbitration_id:02X} not in configured motor_ids {sorted(expected_ids)}'
                        )
                        self.last_unexpected_feedback_log_monotonic = now_sec
                    continue

                feedback = parse_feedback_frame(feedback_msg)
                if feedback is None:
                    if now_sec - self.last_parse_failure_log_monotonic >= 1.0:
                        self.get_logger().warn(
                            f'Failed to parse feedback frame from motor_id=0x{feedback_msg.arbitration_id:02X}'
                        )
                        self.last_parse_failure_log_monotonic = now_sec
                    continue

                feedback_id = int(feedback['id'])
                if feedback_id != motor.config.motor_id:
                    if now_sec - self.last_unexpected_feedback_log_monotonic >= 1.0:
                        self.get_logger().warn(
                            f'Feedback id mismatch for {motor.config.joint_name}: '
                            f'can_id=0x{feedback_msg.arbitration_id:02X}, payload_id=0x{feedback_id:02X}'
                        )
                        self.last_unexpected_feedback_log_monotonic = now_sec
                    continue

                motor.feedback_to_joint_state(feedback)
                pending_ids.discard(motor.config.motor_id)

            for motor in active_motors:
                if now_sec - motor.state.last_feedback_monotonic > 2.0:
                    if now_sec - motor.last_feedback_warn_monotonic > 2.0:
                        self.get_logger().warn(
                            f'No recent feedback received for {motor.config.joint_name} (motor_id=0x{motor.config.motor_id:02X})'
                        )
                        motor.last_feedback_warn_monotonic = now_sec

    def publish_state(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [motor.config.joint_name for motor in self.motors]
        msg.position = [motor.state.position for motor in self.motors]
        msg.velocity = [motor.state.velocity for motor in self.motors]
        msg.effort = [motor.state.effort for motor in self.motors]
        self.state_pub.publish(msg)

        detailed_msg = MotorStateArray()
        detailed_msg.stamp = msg.header.stamp
        detailed_msg.motors = []
        now_sec = time.monotonic()
        for motor in self.motors:
            state_msg = MotorState()
            state_msg.motor_id = int(motor.config.motor_id)
            state_msg.joint_name = motor.config.joint_name
            state_msg.target_position = float(motor.target_position)
            state_msg.position = float(motor.state.position)
            state_msg.velocity = float(motor.state.velocity)
            state_msg.effort = float(motor.state.effort)
            state_msg.feedback_ok = (now_sec - motor.state.last_feedback_monotonic) <= 2.0
            state_msg.enabled = bool(motor.enabled)
            detailed_msg.motors.append(state_msg)
        self.motor_state_pub.publish(detailed_msg)

        if self.legacy_state_pub is not None and self.legacy_motor is not None:
            legacy_msg = Float64()
            legacy_msg.data = float(self.legacy_motor.state.position)
            self.legacy_state_pub.publish(legacy_msg)

    def control_timer_callback(self) -> None:
        self.send_targets()
        self.poll_feedback()
        self.publish_state()

    def shutdown_driver(self) -> None:
        self.get_logger().info('Shutting down AK70 multi driver')
        try:
            if any(motor.enabled for motor in self.motors):
                self.exit_motor_mode_all()
                time.sleep(0.1)
        finally:
            self.bus_collection.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AK70DriverNode()
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
