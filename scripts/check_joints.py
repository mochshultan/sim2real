#!/usr/bin/env python3
"""
NXP Jaguar Joint & RS00 Motor Standalone Passive Diagnostic Tool.
Connects directly to CAN bus in 100% PASSIVE SENSING mode (Kp=0, Kd=0).
No torque is applied to motors, allowing safe manual movement and encoder verification.
"""

import sys
import time
import atexit
import threading
import numpy as np

import parameters as P
from xiaomimotor_lib import CanMotorController

class PassiveJointChecker:
    def __init__(self):
        self.motors = [None] * P.N_JOINTS
        self.joint_pos = [0.0] * P.N_JOINTS
        self.joint_vel = [0.0] * P.N_JOINTS
        self.joint_tau = [0.0] * P.N_JOINTS
        self.joint_tem = [0.0] * P.N_JOINTS
        self.running = True
        self.lock = threading.Lock()

        atexit.register(self.shutdown)

        # Initialize CAN motors per bus
        bus_list = sorted(list(set(P.DEVICE)))
        print(f"Initializing CAN buses: {bus_list} in 100% PASSIVE mode...")

        for bus_name in bus_list:
            indices = [i for i, dev in enumerate(P.DEVICE) if dev == bus_name]
            for i in indices:
                try:
                    self.motors[i] = CanMotorController(
                        bus=P.DEVICE[i],
                        motor_id=P.CAN_ID[i],
                        motor_type=P.MOTOR_TYPE[i],
                        motor_dir=P.MOTOR_DIR[i]
                    )
                    self.motors[i].enable_motor()
                    time.sleep(0.02)
                    self.motors[i].set_run_mode("CONTROL_MODE")
                except Exception as e:
                    print(f"Error connecting to Motor #{P.CAN_ID[i]} on {bus_name}: {e}")

        # Start poll thread
        self.poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
        self.poll_thread.start()

    def _poll_loop(self):
        while self.running:
            for i in range(P.N_JOINTS):
                motor = self.motors[i]
                if motor is not None:
                    try:
                        # 100% PASSIVE: Zero torque, Kp=0, Kd=0
                        can_id, pos, vel, tau, tem = motor.send_control_command(
                            p_ref=0.0, v_ref=0.0, kp=0.0, kd=0.0, tau_ff=0.0
                        )
                        if pos is not None:
                            with self.lock:
                                self.joint_pos[i] = pos
                                self.joint_vel[i] = vel
                                self.joint_tau[i] = tau
                                self.joint_tem[i] = tem
                    except Exception:
                        pass
            time.sleep(0.05)  # 20 Hz display refresh

    def run(self):
        try:
            while self.running:
                sys.stdout.write("\033[2J\033[H")
                print("=" * 88)
                print(" 🐾 NXP JAGUAR: ROBSTRIDE RS00 DIRECT CAN PASSIVE DIAGNOSTIC (Kp=0, Kd=0)")
                print("=" * 88)
                print(f" {'ID':<4} {'Bus':<6} {'Joint Name':<20} {'Motor Type':<13} {'Angle (rad)':<15} {'Velocity':<12} {'Temp'}")
                print("-" * 88)

                with self.lock:
                    for i in range(P.N_JOINTS):
                        cid = P.CAN_ID[i]
                        dev = P.DEVICE[i]
                        jname = P.JOINT_NAME[i]
                        mtype = P.MOTOR_TYPE[i]
                        curr_pos = f"{self.joint_pos[i]:+8.4f} rad"
                        curr_vel = f"{self.joint_vel[i]:+6.2f} r/s"
                        curr_tem = f"{self.joint_tem[i]:.1f}°C"
                        print(f" #{cid:<3} {dev:<6} {jname:<20} {mtype:<13} {curr_pos:<15} {curr_vel:<12} {curr_tem}")

                print("=" * 88)
                print(" [SAFE SENSING MODE] Motors are completely passive (zero torque).")
                print(" Move each leg manually with your hand to verify angle changes. Press Ctrl+C to exit.")
                time.sleep(0.2)
        except KeyboardInterrupt:
            self.shutdown()

    def shutdown(self):
        self.running = False
        print("\nDisabling all motors...")
        for motor in self.motors:
            if motor is not None:
                try:
                    motor.disable_motor()
                except Exception:
                    pass
        print("All motors safely disabled.")

def main():
    checker = PassiveJointChecker()
    checker.run()

if __name__ == "__main__":
    main()
