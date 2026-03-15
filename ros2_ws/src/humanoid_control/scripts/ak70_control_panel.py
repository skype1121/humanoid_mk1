#!/usr/bin/env python3

import math
import os
import shutil
import subprocess
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
    UI_MIN_DEG = 0.0
    UI_MAX_DEG = 360.0
    STATE_STALE_TIMEOUT_SEC = 1.0

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

        self.motor_count = len(self.joint_names)
        if len(self.motor_ids) != self.motor_count:
            raise ValueError('motor_ids length must match joint_names length')
        if len(self.min_positions) != self.motor_count or len(self.max_positions) != self.motor_count:
            raise ValueError('min_positions and max_positions length must match joint_names length')
        if len(self.zero_pose) != self.motor_count:
            self.zero_pose = [0.0] * self.motor_count
        if len(set(self.joint_names)) != self.motor_count:
            raise ValueError('joint_names must be unique')
        if len(set(self.motor_ids)) != self.motor_count:
            raise ValueError('motor_ids must be unique')

        self.joint_index_by_name = {
            joint_name: index for index, joint_name in enumerate(self.joint_names)
        }
        if not self.sine_joint_name:
            self.sine_joint_name = self.joint_names[0]
        elif self.sine_joint_name not in self.joint_index_by_name:
            self.get_logger().warn(
                f'sine_joint_name "{self.sine_joint_name}" not found; falling back to {self.joint_names[0]}'
            )
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
        self.last_state_update_by_joint = {}
        self.last_state_message_monotonic = 0.0
        self.slider_vars = {}
        self.entry_vars = {}
        self.position_labels = {}
        self.velocity_labels = {}
        self.effort_labels = {}
        self.status_labels = {}
        self.summary_status_labels = {}
        self.refresh_health_label = None
        self.sine_enabled = False
        self.sine_phase_start = time.monotonic()

        self.root = tk.Tk()
        self.root.title('AK70 Control Panel')
        self.root.protocol('WM_DELETE_WINDOW', self.handle_close)
        self.all_same_var = tk.StringVar(value='0.0')
        self.build_ui()

        self.ui_period_ms = int(1000.0 / self.update_rate_hz)
        self.root.after(self.ui_period_ms, self.ui_timer_callback)
        self.get_logger().info(
            f'AK70 control panel started | joints={self.joint_names}, sine_joint={self.sine_joint_name}'
        )

    def rad_to_deg(self, value_rad: float) -> float:
        return math.degrees(value_rad)

    def deg_to_rad(self, value_deg: float) -> float:
        return math.radians(value_deg)

    def clamp_target_rad(self, joint_name: str, value_rad: float) -> float:
        index = self.joint_index_by_name[joint_name]
        return max(self.min_positions[index], min(self.max_positions[index], value_rad))

    def clamp_ui_degree(self, value_deg: float) -> float:
        return max(self.UI_MIN_DEG, min(self.UI_MAX_DEG, value_deg))

    def clamp_target(self, joint_name: str, value: float) -> float:
        return self.clamp_target_rad(joint_name, value)

    def build_ui(self) -> None:
        style = ttk.Style()
        style.configure('Header.TLabel', font=('Helvetica', 11, 'bold'))
        style.configure('Value.TLabel', font=('Helvetica', 10))

        container = ttk.Frame(self.root)
        container.grid(row=0, column=0, sticky='nsew')
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        canvas = tk.Canvas(container, highlightthickness=0)
        scrollbar = ttk.Scrollbar(container, orient='vertical', command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)

        scrollable_frame.bind(
            '<Configure>',
            lambda _event: canvas.configure(scrollregion=canvas.bbox('all')),
        )

        canvas.create_window((0, 0), window=scrollable_frame, anchor='nw')
        canvas.configure(yscrollcommand=scrollbar.set)

        container.columnconfigure(0, weight=1)
        container.rowconfigure(0, weight=1)
        canvas.grid(row=0, column=0, sticky='nsew')
        scrollbar.grid(row=0, column=1, sticky='ns')

        scrollable_frame.bind(
            '<Enter>',
            lambda _event: canvas.bind_all(
                '<MouseWheel>',
                lambda event: canvas.yview_scroll(int(-event.delta / 120), 'units'),
            ),
        )
        scrollable_frame.bind(
            '<Leave>',
            lambda _event: canvas.unbind_all('<MouseWheel>'),
        )

        header = ttk.Frame(scrollable_frame, padding=12)
        header.grid(row=0, column=0, sticky='ew')
        header.columnconfigure(0, weight=1)

        ttk.Label(header, text='AK70 Multi Motor Debug Panel (0-360 deg)', style='Header.TLabel').grid(
            row=0, column=0, sticky='w'
        )

        summary_row = ttk.LabelFrame(scrollable_frame, text='Motor Connection / On State', padding=12)
        summary_row.grid(row=1, column=0, sticky='ew', padx=12, pady=(0, 12))
        summary_row.columnconfigure(0, weight=1)
        summary_row.columnconfigure(1, weight=1)

        for row_index in range(5):
            left_index = row_index
            right_index = row_index + 5

            left_joint_name = self.joint_names[left_index]
            left_motor_id = self.motor_ids[left_index]
            left_cell = ttk.LabelFrame(summary_row, text=f'ID {left_motor_id}', padding=8)
            left_cell.grid(row=row_index, column=0, sticky='ew', padx=(0, 6), pady=4)
            left_cell.columnconfigure(1, weight=1)
            ttk.Label(left_cell, text='State:', style='Header.TLabel').grid(row=0, column=0, sticky='w')
            self.summary_status_labels[left_joint_name] = ttk.Label(left_cell, text='X / X', style='Value.TLabel')
            self.summary_status_labels[left_joint_name].grid(row=0, column=1, sticky='w', padx=(8, 0))

            if right_index < self.motor_count:
                right_joint_name = self.joint_names[right_index]
                right_motor_id = self.motor_ids[right_index]
                right_cell = ttk.LabelFrame(summary_row, text=f'ID {right_motor_id}', padding=8)
                right_cell.grid(row=row_index, column=1, sticky='ew', padx=(6, 0), pady=4)
                right_cell.columnconfigure(1, weight=1)
                ttk.Label(right_cell, text='State:', style='Header.TLabel').grid(row=0, column=0, sticky='w')
                self.summary_status_labels[right_joint_name] = ttk.Label(right_cell, text='X / X', style='Value.TLabel')
                self.summary_status_labels[right_joint_name].grid(row=0, column=1, sticky='w', padx=(8, 0))

        refresh_row = ttk.Frame(scrollable_frame, padding=(12, 0, 12, 12))
        refresh_row.grid(row=2, column=0, sticky='ew')
        refresh_row.columnconfigure(1, weight=1)
        ttk.Label(refresh_row, text='Refresh:', style='Header.TLabel').grid(row=0, column=0, sticky='w')
        self.refresh_health_label = ttk.Label(refresh_row, text='X (waiting for state)', style='Value.TLabel')
        self.refresh_health_label.grid(row=0, column=1, sticky='w', padx=(8, 0))

        button_row = ttk.Frame(scrollable_frame, padding=(12, 0, 12, 12))
        button_row.grid(row=3, column=0, sticky='ew')
        for column in range(7):
            button_row.columnconfigure(column, weight=1)

        ttk.Button(button_row, text='Enable All', command=self.enable_all).grid(row=0, column=0, sticky='ew', padx=2)
        ttk.Button(button_row, text='Disable All', command=self.disable_all).grid(row=0, column=1, sticky='ew', padx=2)
        ttk.Button(button_row, text='Send All', command=self.send_all_joints).grid(row=0, column=2, sticky='ew', padx=2)
        ttk.Button(button_row, text='Zero Pose', command=self.zero_pose_command).grid(row=0, column=3, sticky='ew', padx=2)
        ttk.Button(button_row, text='Stop', command=self.stop_all).grid(row=0, column=4, sticky='ew', padx=2)
        ttk.Button(button_row, text='Test Sine', command=self.toggle_sine).grid(row=0, column=5, sticky='ew', padx=2)
        ttk.Button(button_row, text='Open rqt_graph', command=self.open_rqt_graph).grid(
            row=0,
            column=6,
            sticky='ew',
            padx=2,
        )

        all_same_row = ttk.LabelFrame(
            scrollable_frame,
            text=f'Same Position For All {self.motor_count} Motors (deg)',
            padding=12,
        )
        all_same_row.grid(row=4, column=0, sticky='ew', padx=12, pady=(0, 12))
        all_same_row.columnconfigure(1, weight=1)

        ttk.Label(all_same_row, text='Target 0-360:', style='Header.TLabel').grid(row=0, column=0, sticky='w')
        ttk.Entry(all_same_row, textvariable=self.all_same_var, width=10).grid(row=0, column=1, sticky='w', padx=(8, 8))
        ttk.Button(all_same_row, text='Send Same To All', command=self.send_same_to_all_joints).grid(
            row=0, column=2, sticky='e'
        )

        panel = ttk.Frame(scrollable_frame, padding=12)
        panel.grid(row=5, column=0, sticky='nsew')
        scrollable_frame.columnconfigure(0, weight=1)
        panel.columnconfigure(0, weight=1)
        panel.columnconfigure(1, weight=1)

        for row_index, joint_name in enumerate(self.joint_names):
            motor_id = self.motor_ids[row_index]
            grid_row = row_index % 5
            grid_column = row_index // 5
            row = ttk.LabelFrame(panel, text=f'{joint_name} (ID {motor_id})', padding=10)
            row.grid(row=grid_row, column=grid_column, sticky='nsew', pady=4, padx=4)
            row.columnconfigure(1, weight=1)

            slider_var = tk.DoubleVar(value=0.0)
            entry_var = tk.StringVar(value='0.0')
            self.slider_vars[joint_name] = slider_var
            self.entry_vars[joint_name] = entry_var

            slider = ttk.Scale(
                row,
                from_=self.UI_MIN_DEG,
                to=self.UI_MAX_DEG,
                variable=slider_var,
                command=lambda _value, name=joint_name: self.sync_entry_from_slider(name),
            )
            slider.grid(row=0, column=0, columnspan=2, sticky='ew', pady=(0, 6))

            entry = ttk.Entry(row, textvariable=entry_var, width=10)
            entry.grid(row=1, column=0, sticky='w')
            ttk.Button(
                row,
                text='Send 1',
                command=lambda name=joint_name: self.send_single_joint(name),
            ).grid(row=1, column=1, sticky='e')

            ttk.Label(row, text='Pos (deg):', style='Header.TLabel').grid(row=2, column=0, sticky='w')
            self.position_labels[joint_name] = ttk.Label(row, text='0.0', style='Value.TLabel')
            self.position_labels[joint_name].grid(row=2, column=1, sticky='w')

            ttk.Label(row, text='Vel (deg/s):', style='Header.TLabel').grid(row=3, column=0, sticky='w')
            self.velocity_labels[joint_name] = ttk.Label(row, text='0.0', style='Value.TLabel')
            self.velocity_labels[joint_name].grid(row=3, column=1, sticky='w')

            ttk.Label(row, text='Eff:', style='Header.TLabel').grid(row=4, column=0, sticky='w')
            self.effort_labels[joint_name] = ttk.Label(row, text='0.000', style='Value.TLabel')
            self.effort_labels[joint_name].grid(row=4, column=1, sticky='w')

            ttk.Label(row, text='Status:', style='Header.TLabel').grid(row=5, column=0, sticky='w')
            self.status_labels[joint_name] = ttk.Label(row, text='no feedback', style='Value.TLabel')
            self.status_labels[joint_name].grid(row=5, column=1, sticky='w')

    def sync_entry_from_slider(self, joint_name: str) -> None:
        value_deg = self.clamp_ui_degree(self.slider_vars[joint_name].get())
        self.slider_vars[joint_name].set(value_deg)
        self.entry_vars[joint_name].set(f'{value_deg:.1f}')

    def parse_entry_target(self, joint_name: str) -> float:
        try:
            raw_value_deg = float(self.entry_vars[joint_name].get())
        except ValueError:
            raw_value_deg = self.slider_vars[joint_name].get()

        clamped_deg = self.clamp_ui_degree(raw_value_deg)
        clamped_rad = self.clamp_target_rad(joint_name, self.deg_to_rad(clamped_deg))
        applied_deg = self.rad_to_deg(clamped_rad)

        self.slider_vars[joint_name].set(applied_deg)
        self.entry_vars[joint_name].set(f'{applied_deg:.1f}')
        return clamped_rad

    def parse_all_same_target(self) -> float:
        try:
            raw_value_deg = float(self.all_same_var.get())
        except ValueError:
            raw_value_deg = 0.0

        clamped_deg = self.clamp_ui_degree(raw_value_deg)
        self.all_same_var.set(f'{clamped_deg:.1f}')
        return self.deg_to_rad(clamped_deg)

    def publish_motor_targets(self, targets) -> None:
        message = MotorCommandArray()
        message.stamp = self.get_clock().now().to_msg()
        message.commands = []
        for joint_name, target_position in targets:
            command = MotorCommand()
            command.motor_id = int(self.motor_ids[self.joint_index_by_name[joint_name]])
            command.joint_name = joint_name
            command.target_position = float(self.clamp_target_rad(joint_name, target_position))
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

    def send_all_joints(self) -> None:
        self.sine_enabled = False
        targets = []
        for joint_name in self.joint_names:
            targets.append((joint_name, self.parse_entry_target(joint_name)))
        self.publish_motor_targets(targets)

    def send_same_to_all_joints(self) -> None:
        self.sine_enabled = False
        target_rad = self.parse_all_same_target()
        targets = []
        # 공통 목표값을 각 조인트 입력창에도 반영해서 현재 UI 상태와 실제 전송값을 맞춘다.
        for joint_name in self.joint_names:
            applied_rad = self.clamp_target_rad(joint_name, target_rad)
            applied_deg = self.rad_to_deg(applied_rad)
            self.slider_vars[joint_name].set(applied_deg)
            self.entry_vars[joint_name].set(f'{applied_deg:.1f}')
            targets.append((joint_name, applied_rad))
        self.publish_motor_targets(targets)

    def enable_all(self) -> None:
        self.publish_driver_command('ENABLE_ALL')

    def disable_all(self) -> None:
        self.sine_enabled = False
        self.publish_driver_command('DISABLE_ALL')

    def zero_pose_command(self) -> None:
        self.sine_enabled = False
        for index, joint_name in enumerate(self.joint_names):
            zero_pose_deg = self.clamp_ui_degree(self.rad_to_deg(self.zero_pose[index]))
            self.slider_vars[joint_name].set(zero_pose_deg)
            self.entry_vars[joint_name].set(f'{zero_pose_deg:.1f}')
        self.publish_driver_command('ZERO_POSE', self.joint_names, self.zero_pose)

    def stop_all(self) -> None:
        self.sine_enabled = False
        self.publish_driver_command('STOP')

    def open_rqt_graph(self) -> None:
        # rqt_graph는 Tk 안에 임베드하지 않고 별도 창으로 띄운다.
        display = os.environ.get('DISPLAY', '').strip()
        if not display:
            self.get_logger().error('DISPLAY is not set; cannot open rqt_graph window')
            return

        executable_candidates = [
            '/opt/ros/humble/bin/rqt_graph',
            shutil.which('rqt_graph'),
        ]
        executable = next(
            (
                candidate
                for candidate in executable_candidates
                if candidate and os.path.exists(candidate) and os.access(candidate, os.X_OK)
            ),
            None,
        )

        if executable is None:
            self.get_logger().error('rqt_graph executable was not found')
            return

        env = os.environ.copy()
        env['PATH'] = f"/opt/ros/humble/bin:{env.get('PATH', '')}"
        env.setdefault('AMENT_PREFIX_PATH', '/opt/ros/humble')

        try:
            subprocess.Popen([executable], env=env)
            self.get_logger().info(f'Started rqt_graph via {executable}')
            return
        except OSError as exc:
            self.get_logger().error(f'Failed to start rqt_graph: {exc}')
        except Exception as exc:
            self.get_logger().error(f'Unexpected error while starting rqt_graph: {exc}')

    def toggle_sine(self) -> None:
        self.sine_enabled = not self.sine_enabled
        self.sine_phase_start = time.monotonic()
        if not self.sine_enabled:
            self.stop_all()

    def motor_state_callback(self, msg: MotorStateArray) -> None:
        now_sec = time.monotonic()
        self.last_state_message_monotonic = now_sec
        for motor_state in msg.motors:
            self.current_states[motor_state.joint_name] = motor_state
            self.last_state_update_by_joint[motor_state.joint_name] = now_sec

    def update_state_labels(self) -> None:
        now_sec = time.monotonic()
        for joint_name in self.joint_names:
            motor_state = self.current_states.get(joint_name)
            last_update_sec = self.last_state_update_by_joint.get(joint_name, 0.0)
            is_state_fresh = (now_sec - last_update_sec) <= self.STATE_STALE_TIMEOUT_SEC

            if motor_state is None or not is_state_fresh:
                self.position_labels[joint_name].configure(text='0.0')
                self.velocity_labels[joint_name].configure(text='0.0')
                self.effort_labels[joint_name].configure(text='0.000')
                self.status_labels[joint_name].configure(text='no recent state')
                self.summary_status_labels[joint_name].configure(text='X / X')
                continue
            self.position_labels[joint_name].configure(text=f'{self.rad_to_deg(motor_state.position):+.1f}')
            self.velocity_labels[joint_name].configure(text=f'{self.rad_to_deg(motor_state.velocity):+.1f}')
            self.effort_labels[joint_name].configure(text=f'{motor_state.effort:+.3f}')
            status = 'enabled' if motor_state.enabled else 'disabled'
            if not motor_state.feedback_ok:
                status = f'{status} / no feedback'
            self.status_labels[joint_name].configure(text=status)
            # 상단 요약표는 연결 여부와 enable 여부를 O/X로만 간단히 보여준다.
            conn_mark = 'O' if motor_state.feedback_ok else 'X'
            on_mark = 'O' if motor_state.enabled else 'X'
            self.summary_status_labels[joint_name].configure(text=f'{conn_mark} / {on_mark}')

        refresh_alive = (now_sec - self.last_state_message_monotonic) <= self.STATE_STALE_TIMEOUT_SEC
        refresh_age_sec = now_sec - self.last_state_message_monotonic if self.last_state_message_monotonic > 0.0 else None
        if refresh_alive and refresh_age_sec is not None:
            self.refresh_health_label.configure(text=f'O ({refresh_age_sec:.1f}s ago)')
        elif refresh_age_sec is not None:
            self.refresh_health_label.configure(text=f'X ({refresh_age_sec:.1f}s stale)')
        else:
            self.refresh_health_label.configure(text='X (waiting for state)')

    def publish_sine_command(self) -> None:
        elapsed_sec = time.monotonic() - self.sine_phase_start
        target = self.sine_amplitude_rad * math.sin(2.0 * math.pi * self.sine_frequency_hz * elapsed_sec)
        target = self.clamp_target_rad(self.sine_joint_name, target)
        target_deg = self.clamp_ui_degree(self.rad_to_deg(target))
        self.slider_vars[self.sine_joint_name].set(target_deg)
        self.entry_vars[self.sine_joint_name].set(f'{target_deg:.1f}')
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
