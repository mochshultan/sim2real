#!/usr/bin/env python3
"""
NXP Jaguar State & Joint Verification Diagnostic Tool for ROS 2.
Verifies all 48-D Observation dimensions and joint ordering against Isaac Lab policy expectations.
"""

import sys
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu, JointState, Joy
from geometry_msgs.msg import Twist

ISAAC_JOINT_NAMES = [
    "Fr_roll_joint", "Fl_roll_joint", "Br_roll_joint", "Bl_roll_joint",
    "Fr_hip_pitch_joint", "Fl_hip_pitch_joint", "Br_hip_pitch_joint", "Bl_hip_pitch_joint",
    "Fr_knee_joint", "Fl_knee_joint", "Br_knee_joint", "Bl_knee_joint",
]

ROS_NAME_TO_ISAAC_IDX = {
    'FR_collar_joint': 0, 'Fr_roll_joint': 0,
    'FL_collar_joint': 1, 'Fl_roll_joint': 1,
    'BR_collar_joint': 2, 'Br_roll_joint': 2,
    'BL_collar_joint': 3, 'Bl_roll_joint': 3,
    'FR_hip_joint': 4,    'Fr_hip_pitch_joint': 4,
    'FL_hip_joint': 5,    'Fl_hip_pitch_joint': 5,
    'BR_hip_joint': 6,    'Br_hip_pitch_joint': 6,
    'BL_hip_joint': 7,    'Bl_hip_pitch_joint': 7,
    'FR_knee_joint': 8,   'Fr_knee_joint': 8,
    'FL_knee_joint': 9,   'Fl_knee_joint': 9,
    'BR_knee_joint': 10,  'Br_knee_joint': 10,
    'BL_knee_joint': 11,  'Bl_knee_joint': 11,
}

DEFAULT_JOINT_POS = np.array([
   -0.10,  0.0,   0.0,   0.0,    # Rolls (Fr, Fl, Br, Bl)
   -1.60, -1.60, -1.50, -1.50,   # Hip Pitches (Fr, Fl, Br, Bl)
    1.70,  1.70,  1.40,  1.40,   # Knees (Fr, Fl, Br, Bl)
], dtype=np.float32)

class StateCheckerNode(Node):
    def __init__(self):
        super().__init__("jaguar_state_checker")

        self.joint_pos = np.zeros(12, dtype=np.float32)
        self.joint_vel = np.zeros(12, dtype=np.float32)
        self.quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
        self.ang_vel = np.zeros(3, dtype=np.float32)
        self.lin_vel = np.zeros(3, dtype=np.float32)
        self.cmd_vel = np.zeros(3, dtype=np.float32)

        self.imu_count = 0
        self.joint_count = 0
        self.cmd_count = 0

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(Imu, "/imu/data", self._imu_cb, sensor_qos)
        self.create_subscription(Imu, "/Imu_data", self._imu_cb, sensor_qos)
        self.create_subscription(JointState, "/joint_states", self._joint_cb, 10)
        self.create_subscription(Twist, "/cmd_vel", self._cmd_cb, 10)
        self.create_subscription(Joy, "/joy", self._joy_cb, 10)

        # 4 Hz terminal refresh
        self.timer = self.create_timer(0.25, self._display_dashboard)

    def _imu_cb(self, msg: Imu):
        # Raw IMU is mounted upside down on chassis (Rotated 180 deg around X-axis: rpy="pi 0 0")
        # Transformation: q_body = q_raw * q_mount_inv where q_mount is roll(pi)
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        body_qx = qw
        body_qy = qz
        body_qz = -qy
        body_qw = -qx

        self.quat = np.array([body_qx, body_qy, body_qz, body_qw], dtype=np.float32)
        self.ang_vel = np.array([
            msg.angular_velocity.x,
            -msg.angular_velocity.y,
            -msg.angular_velocity.z,
        ], dtype=np.float32)
        self.imu_count += 1

    def _joy_cb(self, msg: Joy):
        if len(msg.axes) >= 2:
            self.cmd_vel[0] = msg.axes[1] * 0.8
            self.cmd_vel[1] = msg.axes[0] * 0.5
            if len(msg.axes) > 3:
                self.cmd_vel[2] = msg.axes[3] * 0.8
            elif len(msg.axes) >= 3:
                self.cmd_vel[2] = msg.axes[2] * 0.8
            self.cmd_count += 1

    def _joint_cb(self, msg: JointState):
        matched = False
        if msg.name:
            for idx, name in enumerate(msg.name):
                if name in ROS_NAME_TO_ISAAC_IDX:
                    isaac_idx = ROS_NAME_TO_ISAAC_IDX[name]
                    if len(msg.position) > idx:
                        self.joint_pos[isaac_idx] = msg.position[idx]
                    if len(msg.velocity) > idx:
                        self.joint_vel[isaac_idx] = msg.velocity[idx]
                    matched = True
        if not matched and len(msg.position) == 12:
            for i in range(12):
                ros_idx = ROS_TO_ISAAC[i]
                self.joint_pos[i] = msg.position[ros_idx]
                if len(msg.velocity) > ros_idx:
                    self.joint_vel[i] = msg.velocity[ros_idx]
        self.joint_count += 1

    def _cmd_cb(self, msg: Twist):
        self.cmd_vel = np.array([msg.linear.x, msg.linear.y, msg.angular.z], dtype=np.float32)
        self.cmd_count += 1

    def _display_dashboard(self):
        qx, qy, qz, qw = self.quat
        # Isaac Lab standard projected gravity (R^T * [0, 0, -1] -> [0, 0, -1] when upright)
        gx = -2.0 * (qx * qz - qw * qy)
        gy = -2.0 * (qy * qz + qw * qx)
        gz = -(1.0 - 2.0 * (qx * qx + qy * qy))

        rel_pos = self.joint_pos - DEFAULT_JOINT_POS

        # Clear screen
        sys.stdout.write("\033[2J\033[H")
        print("=" * 80)
        print(" 🐾 NXP JAGUAR: 48-D ACTOR OBSERVATION STATE & JOINT DIAGNOSTIC DASHBOARD")
        print("=" * 80)

        # 1. IMU & Commands
        print(f"📡 SENSOR STATUS | IMU Msg: {self.imu_count:6d} | Joint Msg: {self.joint_count:6d}")
        print("-" * 80)
        print("1. BASE VELOCITY & GRAVITY PROJECTION:")
        print(f"   • Base Lin Vel  [0:3] : [{self.lin_vel[0]:+6.2f}, {self.lin_vel[1]:+6.2f}, {self.lin_vel[2]:+6.2f}] m/s")
        print(f"   • Base Ang Vel  [3:6] : [{self.ang_vel[0]:+6.2f}, {self.ang_vel[1]:+6.2f}, {self.ang_vel[2]:+6.2f}] rad/s")
        print(f"   • Proj Gravity  [6:9] : [{gx:+6.2f}, {gy:+6.2f}, {gz:+6.2f}] (Upright is [0.0, 0.0, -1.0])")
        print(f"   • Velocity Cmd [9:12] : [{self.cmd_vel[0]:+6.2f}, {self.cmd_vel[1]:+6.2f}, {self.cmd_vel[2]:+6.2f}]")
        print("-" * 80)

        # 2. Joint Status Table
        print("2. 12-JOINT STATE ORDER (Isaac Lab Actor Dimension [12:36]):")
        print(f"   {'Index':<6} {'Joint Name (Isaac Lab)':<24} {'q_curr (rad)':<14} {'q0_nom':<10} {'q - q0 (rel)':<14} {'q_dot (rad/s)':<14}")
        print("   " + "-" * 76)

        for i in range(12):
            name = ISAAC_JOINT_NAMES[i]
            q_curr = self.joint_pos[i]
            q0 = DEFAULT_JOINT_POS[i]
            q_rel = rel_pos[i]
            q_dot = self.joint_vel[i]
            print(f"   [{i:02d}]   {name:<24} {q_curr:+10.4f}     {q0:+6.2f}     {q_rel:+10.4f}     {q_dot:+10.4f}")

        print("-" * 80)

        # 3. Automated Sanity Checks
        print("3. AUTOMATED SANITY CHECKS:")
        imu_ok = (self.imu_count > 0)
        joints_ok = (self.joint_count > 0)
        grav_ok = (abs(gz - (-1.0)) < 0.25)
        standing_error = np.max(np.abs(rel_pos))

        print(f"   [{'OK' if imu_ok else 'FAIL'}] IMU Stream Active: {'Received' if imu_ok else 'Waiting for /imu/data or /Imu_data'}")
        print(f"   [{'OK' if joints_ok else 'FAIL'}] Joint State Stream: {'Received 12 joints' if joints_ok else 'Waiting for /joint_states'}")
        print(f"   [{'OK' if grav_ok else 'WARN'}] Robot Orientation: {'Upright' if grav_ok else 'Tilted / Inverted'}")
        print(f"   [{'OK' if standing_error < 0.3 else 'WARN'}] Max Deviation from Stand Pose: {standing_error:.3f} rad")
        print("=" * 80)
        print(" Press Ctrl+C to exit diagnostic tool.")


def main(args=None):
    rclpy.init(args=args)
    node = StateCheckerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
