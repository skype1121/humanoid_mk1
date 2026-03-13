import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import can

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


def float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
    span = x_max - x_min
    offset = x - x_min
    max_int = (1 << bits) - 1
    value = int(offset * max_int / span)

    if value < 0:
        return 0
    if value > max_int:
        return max_int
    return value


def uint_to_float(x: int, x_min: float, x_max: float, bits: int) -> float:
    span = x_max - x_min
    max_int = (1 << bits) - 1
    return float(x) * span / max_int + x_min


def pack_mit_command(p: float, v: float, kp: float, kd: float, t_ff: float) -> bytes:
    p_int = float_to_uint(p, P_MIN, P_MAX, 16)
    v_int = float_to_uint(v, V_MIN, V_MAX, 12)
    kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12)
    kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12)
    t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12)

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


def parse_feedback_frame(msg: can.Message) -> Optional[Dict[str, float]]:
    # MIT 피드백 프레임을 위치/속도/토크로 디코딩한다.
    if len(msg.data) < 6:
        return None

    data = msg.data

    p_int = (data[1] << 8) | data[2]
    v_int = (data[3] << 4) | (data[4] >> 4)
    t_int = ((data[4] & 0x0F) << 8) | data[5]

    return {
        'id': data[0],
        'pos': uint_to_float(p_int, P_MIN, P_MAX, 16),
        'vel': uint_to_float(v_int, V_MIN, V_MAX, 12),
        'torque': uint_to_float(t_int, T_MIN, T_MAX, 12),
    }


@dataclass
class AK70MotorConfig:
    joint_name: str
    motor_id: int
    can_interface: str
    direction: float = 1.0
    zero_offset: float = 0.0
    kp: float = 12.0
    kd: float = 1.0
    min_position: float = P_MIN
    max_position: float = P_MAX
    max_step_per_cycle: float = 0.05


@dataclass
class AK70MotorState:
    position: float = 0.0
    velocity: float = 0.0
    effort: float = 0.0
    last_feedback_monotonic: float = 0.0
    feedback_count: int = 0


@dataclass
class AK70Motor:
    config: AK70MotorConfig
    target_position: float = 0.0
    last_sent_target_position: float = 0.0
    last_command_monotonic: Optional[float] = None
    last_feedback_warn_monotonic: float = 0.0
    last_timeout_warn_monotonic: float = 0.0
    enabled: bool = False
    state: AK70MotorState = field(default_factory=AK70MotorState)

    def clamp_joint_position(self, joint_position: float) -> float:
        return max(self.config.min_position, min(self.config.max_position, joint_position))

    def limit_target_step(self, desired_position: float) -> float:
        clamped_position = self.clamp_joint_position(desired_position)
        step_limit = max(self.config.max_step_per_cycle, 0.0)
        if step_limit <= 0.0:
            return clamped_position

        lower_bound = self.last_sent_target_position - step_limit
        upper_bound = self.last_sent_target_position + step_limit
        return max(lower_bound, min(upper_bound, clamped_position))

    def command_to_motor_position(self, joint_position: float) -> float:
        # 가정 기반 구현:
        # zero_offset은 "조인트 0rad에서의 모터 절대 위치"로 간주한다.
        return self.config.zero_offset + self.config.direction * joint_position

    def feedback_to_joint_state(self, feedback: Dict[str, float]) -> None:
        # 가정 기반 구현:
        # direction은 조인트 좌표계와 모터 좌표계의 부호 차이를 보정한다.
        self.state.position = self.config.direction * (float(feedback['pos']) - self.config.zero_offset)
        self.state.velocity = self.config.direction * float(feedback['vel'])
        self.state.effort = self.config.direction * float(feedback['torque'])
        self.state.last_feedback_monotonic = time.monotonic()
        self.state.feedback_count += 1


class AK70BusCollection:
    def __init__(self, interface_names: List[str]) -> None:
        self._buses: Dict[str, can.BusABC] = {}
        for interface_name in interface_names:
            if interface_name in self._buses:
                continue
            self._buses[interface_name] = can.Bus(interface='socketcan', channel=interface_name)

    def send(self, interface_name: str, motor_id: int, data: bytes) -> None:
        msg = can.Message(
            arbitration_id=motor_id,
            data=data,
            is_extended_id=False,
        )
        self._buses[interface_name].send(msg)

    def recv(self, interface_name: str, timeout: float) -> Optional[can.Message]:
        return self._buses[interface_name].recv(timeout=timeout)

    def shutdown(self) -> None:
        for bus in self._buses.values():
            bus.shutdown()
