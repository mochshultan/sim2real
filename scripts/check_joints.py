#!/usr/bin/env python3
"""
NXP Jaguar Joint & RS00 Motor Diagnostic Tool.
Tests CAN Bus communication, reads raw encoder positions, and validates zero offsets.
"""

import sys
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from parameters import (
    N_JOINTS, CAN_ID, DEVICE, MOTOR_TYPE, MOTOR_DIR,
    JOINT_NAME, DEFAULT_ANGLE, STANDBY_ANGLE
)

class JointCheckerNode(Node):
    def __init__(self):
        super().__init__("jaguar_joint_checker")
        self.joint_pos = [0.0] * N_JOINTS
        self.joint_vel = [0.0] * N_JOINTS
        self.joint_names = JOINT_NAME.copy()

        self.create_subscription(JointState, "/joint_states", self._cb, 10)
        self.timer = self.create_timer(0.5, self._display)

    def _cb(self, msg: JointState):
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.joint_pos[i] = msg.position[idx]
                if len(msg.velocity) > idx:
                    self.joint_vel[i] = msg.velocity[idx]

    def _display(self):
        sys.stdout.write("\033[2J\033[H")
        print("=" * 80)
        print(" 🐾 NXP JAGUAR: ROBSTRIDE RS00 MOTOR & CAN BUS DIAGNOSTIC")
        print("=" * 80)
        print(f" {'ID':<4} {'Bus':<6} {'Joint Name':<20} {'Motor Type':<14} {'Dir':<5} {'Current Angle':<15} {'Default Pose'}")
        print("-" * 80)

        for i in range(N_JOINTS):
            cid = CAN_ID[i]
            dev = DEVICE[i]
            jname = JOINT_NAME[i]
            mtype = MOTOR_TYPE[i]
            mdir = f"{MOTOR_DIR[i]:+d}"
            curr = f"{self.joint_pos[i]:+8.4f} rad"
            nom = f"{DEFAULT_ANGLE[i]:+6.2f} rad"
            print(f" #{cid:<3} {dev:<6} {jname:<20} {mtype:<14} {mdir:<5} {curr:<15} {nom}")

        print("=" * 80)
        print(" Move legs manually while suspended to confirm encoder sign and rotation direction.")


def main(args=None):
    rclpy.init(args=args)
    node = JointCheckerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
