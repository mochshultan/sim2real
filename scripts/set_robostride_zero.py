#!/usr/bin/env python3
"""
RobStride Motor Mechanical Zero Calibration Utility.
Sends Type 6 mechanical zero write commands to RobStride RS00 motors across can0 and can1.
"""

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
    RobStride private protocol:
    Communication type 6 = Set motor mechanical zero.

    Request CAN ID:
      0x06 00 FE <motor_id>
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
    """Waits for zero acknowledgement reply from target motor."""
    deadline = time.time() + timeout_s

    while time.time() < deadline:
        rx = bus.recv(timeout=0.01)
        if rx is None or not rx.is_extended_id:
            continue

        comm_type, mid_field, low_byte = parse_reply(rx.arbitration_id)
        reply_motor_id = mid_field & 0xFF

        if low_byte == HOST_ID and reply_motor_id == motor_id:
            print(
                f"    Reply: ID=0x{rx.arbitration_id:08X} "
                f"Data={bytes(rx.data).hex(' ').upper()}"
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
                f"[SEND] {channel} Motor #{motor_id}: "
                f"ID=0x{msg.arbitration_id:08X} "
                f"Data={bytes(msg.data).hex(' ').upper()}"
            )

            try:
                bus.send(msg, timeout=0.1)
            except can.CanError as e:
                print(f"[ERROR] {channel} Motor #{motor_id}: Send failed: {e}")
                continue

            got_reply = wait_for_reply(bus, motor_id)
            if got_reply:
                print(f"[OK] {channel} Motor #{motor_id}: Zero command acknowledged.")
            else:
                print(f"[WARN] {channel} Motor #{motor_id}: No reply detected.")

            time.sleep(0.05)


def main() -> None:
    print("==================================================")
    print(" 🐾 RobStride RS00 Mechanical Zero Calibration")
    print("==================================================")
    print("This will set the CURRENT physical position as zero in flash memory.")
    print("Ensure the robot is placed in the calibrated reference pose.\n")
    print("Target motors:")
    for channel, ids in MOTORS_BY_CHANNEL.items():
        print(f"  {channel}: IDs {ids}")

    print()
    input("Place robot in Relax/Zero pose, then press ENTER to proceed...")

    for channel, motor_ids in MOTORS_BY_CHANNEL.items():
        zero_channel(channel, motor_ids)

    print("\n✅ Zero calibration completed.")


if __name__ == "__main__":
    main()