import time
import math
import can


CHANNEL = "can0"
MOTOR_ID = 0x05

# MIT protocol common ranges
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
        value = 0
    if value > max_int:
        value = max_int
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


def send_frame(bus: can.BusABC, arb_id: int, data: bytes) -> None:
    msg = can.Message(
        arbitration_id=arb_id,
        data=data,
        is_extended_id=False,
    )
    bus.send(msg)


def enter_motor_mode(bus: can.BusABC) -> None:
    send_frame(bus, MOTOR_ID, bytes.fromhex("FF FF FF FF FF FF FF FC"))


def exit_motor_mode(bus: can.BusABC) -> None:
    send_frame(bus, MOTOR_ID, bytes.fromhex("FF FF FF FF FF FF FF FD"))


def zero_command(bus: can.BusABC, repeat: int = 50, dt: float = 0.01) -> None:
    data = pack_mit_command(
        p=0.0,
        v=0.0,
        kp=0.0,
        kd=0.0,
        t_ff=0.0,
    )
    for _ in range(repeat):
        send_frame(bus, MOTOR_ID, data)
        time.sleep(dt)


def send_position(bus: can.BusABC, target_pos: float, kp: float = 12.0, kd: float = 1.0) -> None:
    data = pack_mit_command(
        p=target_pos,
        v=0.0,
        kp=kp,
        kd=kd,
        t_ff=0.0,
    )
    send_frame(bus, MOTOR_ID, data)


def parse_feedback(msg: can.Message):
    """
    Common MIT feedback frame format:
    byte0 : id
    byte1-2 : position (16bit)
    byte3 + upper nibble byte4 : velocity (12bit)
    lower nibble byte4 + byte5 : torque/current (12bit)
    """
    if len(msg.data) < 6:
        return None

    data = msg.data

    p_int = (data[1] << 8) | data[2]
    v_int = (data[3] << 4) | (data[4] >> 4)
    t_int = ((data[4] & 0x0F) << 8) | data[5]

    pos = uint_to_float(p_int, P_MIN, P_MAX, 16)
    vel = uint_to_float(v_int, V_MIN, V_MAX, 12)
    torque = uint_to_float(t_int, T_MIN, T_MAX, 12)

    return {
        "id": data[0],
        "pos": pos,
        "vel": vel,
        "torque": torque,
    }


def read_feedback(bus: can.BusABC, timeout: float = 0.02):
    msg = bus.recv(timeout=timeout)
    if msg is None:
        return None

    # motor feedback id may be MOTOR_ID or encoded in data[0], depending on firmware
    if msg.arbitration_id != MOTOR_ID:
        # 그래도 일단 파싱 시도 가능하면 할 수 있지만, 여기선 같은 ID만 받음
        return None

    return parse_feedback(msg)


def print_feedback(prefix: str, fb):
    if fb is None:
        print(f"{prefix} | feedback: timeout")
        return

    print(
        f"{prefix} | "
        f"pos={fb['pos']:+.3f} rad | "
        f"vel={fb['vel']:+.3f} rad/s | "
        f"torque={fb['torque']:+.3f}"
    )


def move_slow(
    bus: can.BusABC,
    start_pos: float,
    end_pos: float,
    move_time: float,
    dt: float = 0.02,
    print_every: int = 10,
) -> None:
    steps = int(move_time / dt)
    if steps <= 0:
        steps = 1

    for i in range(steps + 1):
        alpha = i / steps
        pos = start_pos + (end_pos - start_pos) * alpha
        send_position(bus, pos, kp=12.0, kd=1.0)

        fb = read_feedback(bus, timeout=0.005)
        if i % print_every == 0:
            print_feedback(f"move {i:04d}/{steps:04d}", fb)

        time.sleep(dt)


def hold_position(
    bus: can.BusABC,
    target_pos: float,
    seconds: float,
    dt: float = 0.02,
    print_every: int = 10,
) -> None:
    steps = int(seconds / dt)
    if steps <= 0:
        steps = 1

    for i in range(steps):
        send_position(bus, target_pos, kp=12.0, kd=1.0)

        fb = read_feedback(bus, timeout=0.005)
        if i % print_every == 0:
            print_feedback(f"hold {i:04d}/{steps:04d}", fb)

        time.sleep(dt)


def main() -> None:
    bus = can.Bus(interface="socketcan", channel=CHANNEL)

    one_turn = 2.0 * math.pi  # 6.283 rad

    try:
        print("1) Enter motor mode")
        enter_motor_mode(bus)
        time.sleep(0.1)

        print("2) Zero command")
        zero_command(bus, repeat=50, dt=0.01)

        print("3) Move slowly forward 1 turn")
        move_slow(bus, start_pos=0.0, end_pos=one_turn, move_time=3.0, dt=0.02, print_every=10)
        hold_position(bus, target_pos=one_turn, seconds=1.0, dt=0.02, print_every=10)

        print("4) Move backward 1 turn")
        move_slow(bus, start_pos=one_turn, end_pos=0.0, move_time=3.0, dt=0.02, print_every=5)
        hold_position(bus, target_pos=0.0, seconds=1.0, dt=0.02, print_every=10)

        print("5) Exit motor mode")
        exit_motor_mode(bus)
        time.sleep(0.1)

        print("Done")

    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()