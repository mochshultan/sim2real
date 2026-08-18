#!/usr/bin/env python3

import time
import can


HOST_ID = 0xFE
BITRATE = 1_000_000

MOTORS_BY_CHANNEL = {
    "can0": [1, 2, 3, 4, 5, 6],
    "can1": [1, 2, 3, 4, 5, 6],
}


def make_zero_msg(motor_id: int) -> can.Message:
    """
    RoboStride private protocol:
    Communication type 6 = Set motor mechanical zero

    Request CAN ID:
      0x06 00 FE <motor_id>

    Example:
      motor_id = 1
      arbitration_id = 0x0600FE01

    Data:
      01 00 00 00 00 00 00 00
    """
    comm_type = 0x06
    arbitration_id = (comm_type << 24) | (HOST_ID << 8) | motor_id

    return can.Message(
        arbitration_id=arbitration_id,
        data=[0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
        is_extended_id=True,
    )


def parse_reply(arbitration_id: int):
    comm_type = (arbitration_id >> 24) & 0x1F
    mid_field = (arbitration_id >> 8) & 0xFFFF
    low_byte = arbitration_id & 0xFF
    return comm_type, mid_field, low_byte


def wait_for_reply(bus: can.BusABC, motor_id: int, timeout_s: float = 0.2) -> bool:
    """
    Zero command should produce feedback/reply from target motor.
    Accept replies where low byte == HOST_ID and motor id appears in mid_field.
    """
    deadline = time.time() + timeout_s

    while time.time() < deadline:
        rx = bus.recv(timeout=0.01)
        if rx is None:
            continue

        if not rx.is_extended_id:
            continue

        comm_type, mid_field, low_byte = parse_reply(rx.arbitration_id)

        reply_motor_id = mid_field & 0xFF

        if low_byte == HOST_ID and reply_motor_id == motor_id:
            print(
                f"    reply: id=0x{rx.arbitration_id:08X} "
                f"data={bytes(rx.data).hex(' ').upper()}"
            )
            return True

    return False


def drain_bus(bus: can.BusABC, duration_s: float = 0.05) -> None:
    end = time.time() + duration_s
    while time.time() < end:
        bus.recv(timeout=0.001)


def zero_channel(channel: str, motor_ids: list[int]) -> None:
    print(f"\nOpening {channel}...")

    with can.interface.Bus(
        channel=channel,
        interface="socketcan",
        bitrate=BITRATE,
    ) as bus:
        drain_bus(bus)

        for motor_id in motor_ids:
            msg = make_zero_msg(motor_id)

            print(
                f"[SEND] {channel} motor {motor_id}: "
                f"id=0x{msg.arbitration_id:08X} "
                f"data={bytes(msg.data).hex(' ').upper()}"
            )

            try:
                bus.send(msg, timeout=0.1)
            except can.CanError as e:
                print(f"[ERROR] {channel} motor {motor_id}: send failed: {e}")
                continue

            got_reply = wait_for_reply(bus, motor_id)

            if got_reply:
                print(f"[OK] {channel} motor {motor_id}: zero command acknowledged")
            else:
                print(f"[WARN] {channel} motor {motor_id}: no reply detected")

            time.sleep(0.05)


def main() -> None:
    print("RoboStride set mechanical zero")
    print()
    print("This will set the CURRENT mechanical position as zero.")
    print("It will NOT move the motors to zero.")
    print()
    print("Target motors:")
    for channel, ids in MOTORS_BY_CHANNEL.items():
        print(f"  {channel}: {ids}")

    print()
    input("Place robot in zero pose, disable torque if needed, then press ENTER...")

    for channel, motor_ids in MOTORS_BY_CHANNEL.items():
        zero_channel(channel, motor_ids)

    print("\nDone.")


if __name__ == "__main__":
    main()