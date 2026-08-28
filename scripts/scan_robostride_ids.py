#!/usr/bin/env python3
"""
RobStride Motor CAN ID Scanner Utility.
Scans can0 and can1 interfaces to discover connected RobStride RS00 motor node IDs.
"""

import time
import can


HOST_ID = 0xFE
BITRATE = 1_000_000
CHANNELS = ["can0", "can1"]
SCAN_IDS = range(1, 128)


def make_get_device_id_frame(motor_id: int, host_id: int = HOST_ID) -> can.Message:
    """
    RobStride private protocol frame:
    Communication mode 0 = Get device ID.

    Extended CAN ID:
      bit 28..24 = communication type = 0x00
      bit 15..8  = host CAN ID (0xFE)
      bit 7..0   = target motor CAN ID
    """
    comm_type = 0x00
    arbitration_id = (comm_type << 24) | (host_id << 8) | motor_id

    return can.Message(
        arbitration_id=arbitration_id,
        data=[0x00] * 8,
        is_extended_id=True,
    )


def parse_robostride_id(arbitration_id: int):
    comm_type = (arbitration_id >> 24) & 0x1F
    upper_16 = (arbitration_id >> 8) & 0xFFFF
    low_8 = arbitration_id & 0xFF
    return comm_type, upper_16, low_8


def scan_channel(channel: str):
    found = {}
    print(f"\nScanning interface {channel}...")

    with can.interface.Bus(
        channel=channel,
        interface="socketcan",
        bitrate=BITRATE,
    ) as bus:
        # Flush existing RX queue buffer
        start = time.time()
        while time.time() - start < 0.1:
            bus.recv(timeout=0.001)

        for motor_id in SCAN_IDS:
            msg = make_get_device_id_frame(motor_id)
            try:
                bus.send(msg, timeout=0.05)
            except can.CanError as e:
                print(f"[ERROR] {channel}: send failed for ID {motor_id}: {e}")
                continue

            deadline = time.time() + 0.03

            while time.time() < deadline:
                rx = bus.recv(timeout=0.005)
                if rx is None or not rx.is_extended_id:
                    continue

                comm_type, upper_16, low_8 = parse_robostride_id(rx.arbitration_id)

                if comm_type == 0x00 and low_8 == HOST_ID:
                    responding_motor_id = upper_16 & 0xFF
                    unique_id = int.from_bytes(rx.data, byteorder="big", signed=False)

                    found[responding_motor_id] = {
                        "unique_id": unique_id,
                        "raw_can_id": rx.arbitration_id,
                        "data": bytes(rx.data),
                    }

            time.sleep(0.005)

    return found


def main():
    all_found = {}

    for channel in CHANNELS:
        try:
            found = scan_channel(channel)
            all_found[channel] = found
        except OSError as e:
            print(f"[ERROR] Cannot open {channel}: {e}")
            all_found[channel] = {}

    print("\n===== Scan Results =====")

    for channel, motors in all_found.items():
        if not motors:
            print(f"{channel}: No motors detected.")
            continue

        print(f"{channel}: Found {len(motors)} motor(s):")
        for motor_id in sorted(motors.keys()):
            info = motors[motor_id]
            print(
                f"  Motor ID={motor_id:3d} | "
                f"Unique ID=0x{info['unique_id']:016X} | "
                f"Reply CAN ID=0x{info['raw_can_id']:08X} | "
                f"Data={info['data'].hex(' ').upper()}"
            )


if __name__ == "__main__":
    main()