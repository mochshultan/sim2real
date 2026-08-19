#!/usr/bin/env python3
"""
RobStride Motor Interactive Test & Diagnostic Tool (MIT Control Mode).
"""

import argparse
import sys
import select
import time
import numpy as np
from robstride_motor_lib import RobStrideMotorController


def main():
    parser = argparse.ArgumentParser(description="RobStride RS00 Motor MIT Mode Diagnostic Tool")
    parser.add_argument("--device", '-d', type=str, default="can0", help="SocketCAN interface name (can0/can1)")
    parser.add_argument("--ids", '-i', type=int, nargs="+", default=[1], help="Motor CAN IDs to test")
    parser.add_argument("--new_id", '-n', type=int, default=None, help="New motor CAN ID to set")
    parser.add_argument("--motor_type", type=str, default="RobStride00", help="Motor model")
    parser.add_argument("--task", '-t', type=str, default="sense", help="Task: [pos, torque, sense, change, passive]")
    parser.add_argument("--value", '-v', type=float, default=0.0, help="Target value (pos [rad] or torque [Nm])")
    parser.add_argument("--kp", type=float, default=10.0, help="Position gain Kp")
    parser.add_argument("--kd", type=float, default=1.0, help="Velocity gain Kd")
    parser.add_argument("--hz", type=int, default=100, help="Control loop rate (Hz)")
    parser.add_argument("--time", type=float, default=10.0, help="Test duration in seconds")
    args = parser.parse_args()

    print(f"=== RobStride RS00 CAN Test ===")
    print(f"Socket: {args.device} | Motor IDs: {args.ids} | Type: {args.motor_type}")

    ids = args.ids
    motors = {}
    for mid in ids:
        motors[mid] = RobStrideMotorController(
            bus=args.device,
            motor_id=mid,
            motor_dir=1,
            motor_type=args.motor_type
        )

    if args.task == "change":
        assert len(motors) == 1, "Change ID task requires exactly one motor ID"
        motor = motors[ids[0]]
        can_id, pos, vel, tau, tem = motor.change_motor_id(new_motor_id=args.new_id)
        print(f"Changed Motor ID {ids[0]} -> {args.new_id} | Pos: {pos}, Temp: {tem}C")
        sys.exit(0)

    print("Enabling Motors and setting MIT CONTROL_MODE...")
    for mid, motor in motors.items():
        can_id, pos, vel, tau, tem = motor.enable_motor()
        print(f"Motor #{mid} -> Pos: {pos:.3f} rad, Vel: {vel:.3f}, Tau: {tau:.3f} Nm, Temp: {tem:.1f} C")
        motor.set_run_mode("CONTROL_MODE")

    start_time = time.time()
    dt = 1.0 / args.hz

    try:
        while (time.time() - start_time) < args.time:
            for mid, motor in motors.items():
                if args.task == "sense" or args.task == "passive":
                    # Zero-torque passive sensing
                    can_id, pos, vel, tau, tem = motor.send_control_command(
                        p_ref=0.0, v_ref=0.0, kp=0.0, kd=0.0, tau_ff=0.0
                    )
                elif args.task == "pos":
                    can_id, pos, vel, tau, tem = motor.send_control_command(
                        p_ref=args.value, v_ref=0.0, kp=args.kp, kd=args.kd, tau_ff=0.0
                    )
                elif args.task == "torque":
                    can_id, pos, vel, tau, tem = motor.send_control_command(
                        p_ref=0.0, v_ref=0.0, kp=0.0, kd=0.0, tau_ff=args.value
                    )
                
                print(f"\r[t={time.time() - start_time:.1f}s] Motor #{mid} | Pos: {pos:.3f} rad | Vel: {vel:.3f} rad/s | Tau: {tau:.3f} Nm | Temp: {tem:.1f}C", end="")
            time.sleep(dt)
    except KeyboardInterrupt:
        pass
    finally:
        print("\nDisabling all motors for safety...")
        for motor in motors.values():
            motor.disable_motor()
        print("Done.")


if __name__ == "__main__":
    main()
