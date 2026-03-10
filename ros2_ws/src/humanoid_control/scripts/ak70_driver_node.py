#!/usr/bin/env python3

import time
from typing import Dict, List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from humanoid_control_py.ak70_driver_core import AK70BusCollection
from humanoid_control_py.ak70_driver_core import AK70Motor
from humanoid_control_py.ak70_driver_core import AK70MotorConfig
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
        self.declare_parameter('joint_names', [])
        self.declare_parameter('motor_ids', [])
        self.declare_parameter('can_interfaces', [])
        self.declare_parameter('directions', [])
        self.declare_parameter('zero_offsets', [])
        self.declare_parameter('kps', [])
        self.declare_parameter('kds', [])

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
            self.command_callback,
            10,
        )
        self.state_pub = self.create_publisher(JointState, self.state_topic, 10)

        if self.send_zero_on_startup and not self.enable_motors_on_startup:
            self.get_logger().warn('send_zero_on_startup=true but enable_motors_on_startup=false; zero commands may be ignored')

        if self.enable_motors_on_startup:
            self.enter_motor_mode_all()
            time.sleep(0.1)
        if self.send_zero_on_startup and self.startup_zero_command_repeat > 0:
            self.zero_command_all(
                repeat=self.startup_zero_command_repeat,
                dt=self.startup_zero_command_dt,
            )

        self.control_timer = self.create_timer(self.control_period_sec, self.control_timer_callback)
        self.get_logger().info(
            f'AK70 multi driver started | motors={len(self.motors)}, interfaces={sorted(self.motors_by_interface.keys())}, '
            f'enable_on_startup={self.enable_motors_on_startup}, zero_on_startup={self.send_zero_on_startup}'
        )

    def load_motor_configs(self) -> List[AK70Motor]:
        joint_names = [str(value) for value in self.get_parameter('joint_names').value]
        motor_ids = [int(value) for value in self.get_parameter('motor_ids').value]
        can_interfaces = [str(value) for value in self.get_parameter('can_interfaces').value]
        directions = [float(value) for value in self.get_parameter('directions').value]
        zero_offsets = [float(value) for value in self.get_parameter('zero_offsets').value]
        kps = [float(value) for value in self.get_parameter('kps').value]
        kds = [float(value) for value in self.get_parameter('kds').value]

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

        if not kps:
            kps = [12.0] * required_length
        if not kds:
            kds = [1.0] * required_length
        if len(kps) != required_length or len(kds) != required_length:
            raise ValueError('kps and kds must be empty or match joint_names length')

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
                    )
                )
            )
        return motors

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

    def exit_motor_mode_all(self) -> None:
        data = bytes.fromhex('FF FF FF FF FF FF FF FD')
        for motor in self.motors:
            self.send_motor_frame(motor, data)

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

    def command_callback(self, msg: JointState) -> None:
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
            motor.target_position = float(msg.position[index])
            motor.last_command_monotonic = now_sec

    def send_targets(self) -> None:
        now_sec = time.monotonic()
        for motor in self.motors:
            if motor.last_command_monotonic is not None:
                elapsed_sec = now_sec - motor.last_command_monotonic
                if elapsed_sec > self.command_timeout_sec:
                    if now_sec - motor.last_timeout_warn_monotonic > 1.0:
                        self.get_logger().warn(
                            f'command timeout for {motor.config.joint_name} ({elapsed_sec:.2f}s): holding last target {motor.target_position:.3f} rad'
                        )
                        motor.last_timeout_warn_monotonic = now_sec

            motor_position = motor.command_to_motor_position(motor.target_position)
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
            expected_ids = {motor.config.motor_id for motor in interface_motors}
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
                    continue

                feedback = parse_feedback_frame(feedback_msg)
                if feedback is None:
                    continue

                motor.feedback_to_joint_state(feedback)
                pending_ids.discard(motor.config.motor_id)

            for motor in interface_motors:
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

    def control_timer_callback(self) -> None:
        self.send_targets()
        self.poll_feedback()
        self.publish_state()

    def shutdown_driver(self) -> None:
        self.get_logger().info('Shutting down AK70 multi driver')
        try:
            if self.enable_motors_on_startup:
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
