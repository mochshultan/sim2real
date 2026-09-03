#!/usr/bin/env python3
"""
RobStride Motor CAN ID Scanner Utility.
Scans can0 and can1 interfaces to discover connected RobStride RS00 motor node IDs,
maps detected IDs to joint names, and reports missing motors against the expected robot topology.
"""

import os
import sys
import time
import can

# Ensure scripts directory is in path for parameters import
SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)

try:
    import parameters as P
    EXPECTED_MOTORS = {}
    for dev, cid, jname in zip(P.DEVICE, P.CAN_ID, P.JOINT_NAME):
        EXPECTED_MOTORS.setdefault(dev, {})[cid] = jname
except Exception:
    # Fallback robot joint mapping if parameters.py cannot be loaded
    EXPECTED_MOTORS = {
        "can0": {
            1: "FR_collar_joint",
            2: "FR_hip_joint",
            3: "FR_knee_joint",
            4: "BR_collar_joint",
            5: "BR_hip_joint",
            6: "BR_knee_joint",
        },
        "can1": {
            1: "FL_collar_joint",
            2: "FL_hip_joint",
            3: "FL_knee_joint",
            4: "BL_collar_joint",
            5: "BL_hip_joint",
            6: "BL_knee_joint",
        },
    }

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

    total_expected = sum(len(m) for m in EXPECTED_MOTORS.values())
    total_found = 0
    all_missing = []

    for channel in CHANNELS:
        motors = all_found.get(channel, {})
        expected_channel = EXPECTED_MOTORS.get(channel, {})
        total_found += len(motors)

        if not motors:
            print(f"\n{channel}: No motors detected.")
        else:
            print(f"\n{channel}: Found {len(motors)} / {len(expected_channel)} motor(s):")
            for motor_id in sorted(motors.keys()):
                info = motors[motor_id]
                joint_name = expected_channel.get(motor_id, "UNKNOWN_JOINT")
                print(
                    f"  [OK] ID={motor_id:2d} | Joint={joint_name:<16} | "
                    f"UID=0x{info['unique_id']:016X} | "
                    f"Reply CAN ID=0x{info['raw_can_id']:08X} | "
                    f"Data={info['data'].hex(' ').upper()}"
                )

        # Check for missing motors on this channel
        missing_ids = sorted(set(expected_channel.keys()) - set(motors.keys()))
        if missing_ids:
            print(f"  [MISSING] {len(missing_ids)} motor(s) not responding on {channel}:")
            for mid in missing_ids:
                jname = expected_channel.get(mid, "UNKNOWN_JOINT")
                print(f"    -> Motor ID={mid:2d} | Expected Joint: {jname}")
                all_missing.append((channel, mid, jname))

if __name__ == "__main__":
    main()
