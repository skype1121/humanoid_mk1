#!/usr/bin/env python3

import math
import time
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from humanoid_control.msg import DriverCommand
from humanoid_control.msg import MotorCommand
from humanoid_control.msg import MotorCommandArray
from humanoid_control.msg import MotorStateArray


class AK70ControlPanel(Node):
    def __init__(self) -> None:
        super().__init__('ak70_control_panel')

        self.declare_parameter('motor_command_topic', '/ak70/motor_commands')
        self.declare_parameter('motor_state_topic', '/ak70/motor_states_detailed')
        self.declare_parameter('driver_command_topic', '/ak70/driver_command')
        self.declare_parameter('update_rate_hz', 20.0)
        self.declare_parameter('sine_amplitude_rad', 0.1)
        self.declare_parameter('sine_frequency_hz', 0.2)
        self.declare_parameter('sine_joint_name', '')
        self.declare_parameter('joint_names', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('motor_ids', Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter('min_positions', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('max_positions', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('zero_pose', Parameter.Type.DOUBLE_ARRAY)

        self.motor_command_topic = str(self.get_parameter('motor_command_topic').value)
        self.motor_state_topic = str(self.get_parameter('motor_state_topic').value)
        self.driver_command_topic = str(self.get_parameter('driver_command_topic').value)
        self.update_rate_hz = max(float(self.get_parameter('update_rate_hz').value), 5.0)
        self.sine_amplitude_rad = abs(float(self.get_parameter('sine_amplitude_rad').value))
        self.sine_frequency_hz = max(float(self.get_parameter('sine_frequency_hz').value), 0.01)
        self.sine_joint_name = str(self.get_parameter('sine_joint_name').value).strip()

        self.joint_names = [str(value) for value in self.get_parameter('joint_names').value]
        self.motor_ids = [int(value) for value in self.get_parameter('motor_ids').value]
        self.min_positions = [float(value) for value in self.get_parameter('min_positions').value]
        self.max_positions = [float(value) for value in self.get_parameter('max_positions').value]
        self.zero_pose = [float(value) for value in self.get_parameter('zero_pose').value]

        if not self.joint_names:
            raise ValueError('joint_names parameter is empty')

        motor_count = len(self.joint_names)
        if len(self.motor_ids) != motor_count:
            raise ValueError('motor_ids length must match joint_names length')
        if len(self.min_positions) != motor_count or len(self.max_positions) != motor_count:
            raise ValueError('min_positions and max_positions length must match joint_names length')
        if len(self.zero_pose) != motor_count:
            self.zero_pose = [0.0] * motor_count
        if not self.sine_joint_name:
            self.sine_joint_name = self.joint_names[0]

        self.command_pub = self.create_publisher(MotorCommandArray, self.motor_command_topic, 10)
        self.driver_command_pub = self.create_publisher(DriverCommand, self.driver_command_topic, 10)
        self.state_sub = self.create_subscription(
            MotorStateArray,
            self.motor_state_topic,
            self.motor_state_callback,
            10,
        )

        self.current_states = {}
        self.slider_vars = {}
        self.entry_vars = {}
        self.position_labels = {}
        self.velocity_labels = {}
        self.effort_labels = {}
        self.status_labels = {}
        self.sine_enabled = False
        self.sine_phase_start = time.monotonic()

        self.root = tk.Tk()
        self.root.title('AK70 Control Panel')
        self.root.protocol('WM_DELETE_WINDOW', self.handle_close)
        self.build_ui()

        self.ui_period_ms = int(1000.0 / self.update_rate_hz)
        self.root.after(self.ui_period_ms, self.ui_timer_callback)
        self.get_logger().info(
            f'AK70 control panel started | joints={self.joint_names}, sine_joint={self.sine_joint_name}'
        )

    def clamp_target(self, joint_name: str, value: float) -> float:
        index = self.joint_names.index(joint_name)
        return max(self.min_positions[index], min(self.max_positions[index], value))

    def build_ui(self) -> None:
        style = ttk.Style()
        style.configure('Header.TLabel', font=('Helvetica', 11, 'bold'))
        style.configure('Value.TLabel', font=('Helvetica', 10))

        header = ttk.Frame(self.root, padding=12)
        header.grid(row=0, column=0, sticky='ew')
        header.columnconfigure(0, weight=1)

        ttk.Label(header, text='AK70 Multi Motor Debug Panel', style='Header.TLabel').grid(
            row=0, column=0, sticky='w'
        )

        button_row = ttk.Frame(self.root, padding=(12, 0, 12, 12))
        button_row.grid(row=1, column=0, sticky='ew')
        for column in range(5):
            button_row.columnconfigure(column, weight=1)

        ttk.Button(button_row, text='Enable All', command=self.enable_all).grid(row=0, column=0, sticky='ew', padx=2)
        ttk.Button(button_row, text='Disable All', command=self.disable_all).grid(row=0, column=1, sticky='ew', padx=2)
        ttk.Button(button_row, text='Zero Pose', command=self.zero_pose_command).grid(row=0, column=2, sticky='ew', padx=2)
        ttk.Button(button_row, text='Stop', command=self.stop_all).grid(row=0, column=3, sticky='ew', padx=2)
        ttk.Button(button_row, text='Test Sine', command=self.toggle_sine).grid(row=0, column=4, sticky='ew', padx=2)

        panel = ttk.Frame(self.root, padding=12)
        panel.grid(row=2, column=0, sticky='nsew')
        self.root.columnconfigure(0, weight=1)

        for row_index, joint_name in enumerate(self.joint_names):
            motor_id = self.motor_ids[row_index]
            row = ttk.LabelFrame(panel, text=f'{joint_name} (ID {motor_id})', padding=10)
            row.grid(row=row_index, column=0, sticky='ew', pady=4)
            row.columnconfigure(1, weight=1)

            slider_var = tk.DoubleVar(value=0.0)
            entry_var = tk.StringVar(value='0.000')
            self.slider_vars[joint_name] = slider_var
            self.entry_vars[joint_name] = entry_var

            slider = ttk.Scale(
                row,
                from_=self.min_positions[row_index],
                to=self.max_positions[row_index],
                variable=slider_var,
                command=lambda _value, name=joint_name: self.sync_entry_from_slider(name),
            )
            slider.grid(row=0, column=0, columnspan=2, sticky='ew', pady=(0, 6))

            entry = ttk.Entry(row, textvariable=entry_var, width=10)
            entry.grid(row=1, column=0, sticky='w')
            ttk.Button(
                row,
                text='Send',
                command=lambda name=joint_name: self.send_single_joint(name),
            ).grid(row=1, column=1, sticky='e')

            ttk.Label(row, text='Pos:', style='Header.TLabel').grid(row=2, column=0, sticky='w')
            self.position_labels[joint_name] = ttk.Label(row, text='0.000', style='Value.TLabel')
            self.position_labels[joint_name].grid(row=2, column=1, sticky='w')

            ttk.Label(row, text='Vel:', style='Header.TLabel').grid(row=3, column=0, sticky='w')
            self.velocity_labels[joint_name] = ttk.Label(row, text='0.000', style='Value.TLabel')
            self.velocity_labels[joint_name].grid(row=3, column=1, sticky='w')

            ttk.Label(row, text='Eff:', style='Header.TLabel').grid(row=4, column=0, sticky='w')
            self.effort_labels[joint_name] = ttk.Label(row, text='0.000', style='Value.TLabel')
            self.effort_labels[joint_name].grid(row=4, column=1, sticky='w')

            ttk.Label(row, text='Status:', style='Header.TLabel').grid(row=5, column=0, sticky='w')
            self.status_labels[joint_name] = ttk.Label(row, text='no feedback', style='Value.TLabel')
            self.status_labels[joint_name].grid(row=5, column=1, sticky='w')

    def sync_entry_from_slider(self, joint_name: str) -> None:
        value = self.slider_vars[joint_name].get()
        self.entry_vars[joint_name].set(f'{value:.3f}')

    def parse_entry_target(self, joint_name: str) -> float:
        try:
            raw_value = float(self.entry_vars[joint_name].get())
        except ValueError:
            raw_value = self.slider_vars[joint_name].get()
        clamped_value = self.clamp_target(joint_name, raw_value)
        self.slider_vars[joint_name].set(clamped_value)
        self.entry_vars[joint_name].set(f'{clamped_value:.3f}')
        return clamped_value

    def publish_motor_targets(self, targets) -> None:
        message = MotorCommandArray()
        message.stamp = self.get_clock().now().to_msg()
        message.commands = []
        for joint_name, target_position in targets:
            command = MotorCommand()
            command.motor_id = int(self.motor_ids[self.joint_names.index(joint_name)])
            command.joint_name = joint_name
            command.target_position = float(self.clamp_target(joint_name, target_position))
            command.kp = 0.0
            command.kd = 0.0
            command.has_gains = False
            message.commands.append(command)
        self.command_pub.publish(message)

    def publish_driver_command(self, command: str, joint_names=None, target_positions=None) -> None:
        message = DriverCommand()
        message.command = command
        message.joint_names = list(joint_names) if joint_names is not None else []
        message.target_positions = list(target_positions) if target_positions is not None else []
        self.driver_command_pub.publish(message)

    def send_single_joint(self, joint_name: str) -> None:
        self.sine_enabled = False
        target = self.parse_entry_target(joint_name)
        self.publish_motor_targets([(joint_name, target)])

    def enable_all(self) -> None:
        self.publish_driver_command('ENABLE_ALL')

    def disable_all(self) -> None:
        self.sine_enabled = False
        self.publish_driver_command('DISABLE_ALL')

    def zero_pose_command(self) -> None:
        self.sine_enabled = False
        for index, joint_name in enumerate(self.joint_names):
            self.slider_vars[joint_name].set(self.zero_pose[index])
            self.entry_vars[joint_name].set(f'{self.zero_pose[index]:.3f}')
        self.publish_driver_command('ZERO_POSE', self.joint_names, self.zero_pose)

    def stop_all(self) -> None:
        self.sine_enabled = False
        self.publish_driver_command('STOP')

    def toggle_sine(self) -> None:
        self.sine_enabled = not self.sine_enabled
        self.sine_phase_start = time.monotonic()
        if not self.sine_enabled:
            self.stop_all()

    def motor_state_callback(self, msg: MotorStateArray) -> None:
        for motor_state in msg.motors:
            self.current_states[motor_state.joint_name] = motor_state

    def update_state_labels(self) -> None:
        for joint_name in self.joint_names:
            motor_state = self.current_states.get(joint_name)
            if motor_state is None:
                continue
            self.position_labels[joint_name].configure(text=f'{motor_state.position:+.3f}')
            self.velocity_labels[joint_name].configure(text=f'{motor_state.velocity:+.3f}')
            self.effort_labels[joint_name].configure(text=f'{motor_state.effort:+.3f}')
            status = 'enabled' if motor_state.enabled else 'disabled'
            if not motor_state.feedback_ok:
                status = f'{status} / no feedback'
            self.status_labels[joint_name].configure(text=status)

    def publish_sine_command(self) -> None:
        elapsed_sec = time.monotonic() - self.sine_phase_start
        target = self.sine_amplitude_rad * math.sin(2.0 * math.pi * self.sine_frequency_hz * elapsed_sec)
        target = self.clamp_target(self.sine_joint_name, target)
        self.slider_vars[self.sine_joint_name].set(target)
        self.entry_vars[self.sine_joint_name].set(f'{target:.3f}')
        self.publish_motor_targets([(self.sine_joint_name, target)])

    def ui_timer_callback(self) -> None:
        rclpy.spin_once(self, timeout_sec=0.0)
        self.update_state_labels()
        if self.sine_enabled:
            self.publish_sine_command()
        self.root.after(self.ui_period_ms, self.ui_timer_callback)

    def handle_close(self) -> None:
        self.sine_enabled = False
        self.publish_driver_command('STOP')
        self.root.quit()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AK70ControlPanel()
    try:
        node.root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
