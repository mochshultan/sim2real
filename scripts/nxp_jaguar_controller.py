#!/usr/bin/env python3
"""
NXP Jaguar Sim-to-Real Controller Node for ROS 2.
Deploys Isaac Lab 3.0 TorchScript Policy (DreamWaQ) to RobStride RS00 Hardware.
"""

import os
import math
import time
import threading
import numpy as np
import torch

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu, Joy, JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, String

# ==============================================================================
# 1. NXP JAGUAR JOINT ORDER & NOMINAL CONFIGURATION
# ==============================================================================
N_JOINTS = 12

# Isaac Lab 3.0 Joint Order (Grouped by Joint Type)
ISAAC_JOINT_NAMES = [
    "Fr_roll_joint", "Fl_roll_joint", "Br_roll_joint", "Bl_roll_joint",
    "Fr_hip_pitch_joint", "Fl_hip_pitch_joint", "Br_hip_pitch_joint", "Bl_hip_pitch_joint",
    "Fr_knee_joint", "Fl_knee_joint", "Br_knee_joint", "Bl_knee_joint",
]

# ROS CAN Hardware Joint Names (Grouped by Leg: BL, BR, FL, FR)
ROS_JOINT_NAMES = [
    'BL_collar_joint', 'BL_hip_joint', 'BL_knee_joint',
    'BR_collar_joint', 'BR_hip_joint', 'BR_knee_joint',
    'FL_collar_joint', 'FL_hip_joint', 'FL_knee_joint',
    'FR_collar_joint', 'FR_hip_joint', 'FR_knee_joint',
]

# Remapping Indices: ROS CAN Order (BL, BR, FL, FR) <-> Isaac Lab Order
ROS_TO_ISAAC = [9, 6, 3, 0, 10, 7, 4, 1, 11, 8, 5, 2]
ISAAC_TO_ROS = [3, 7, 11, 2, 6, 10, 1, 5, 9, 0, 4, 8]

# Standby Joint Angles (Folded/Sitting position = 0.0 rad)
SIT_JOINT_POS = np.zeros(12, dtype=np.float32)

# Default Standing Pose (Matching NXP_JAGUAR_CFG in Isaac Lab: Front Hips -1.5, Back Hips -1.4, Front Knees 1.4, Back Knees 1.6)
DEFAULT_JOINT_POS = np.array([
    0.0,   0.0,   0.0,   0.0,    # Rolls (Fr, Fl, Br, Bl)
   -1.50, -1.50, -1.40, -1.40,   # Hip Pitches (Fr, Fl, Br, Bl)
    1.40,  1.40,  1.60,  1.60,   # Knees (Fr, Fl, Br, Bl)
], dtype=np.float32)

ACTION_SCALE = 0.25      # Policy action scaling factor
CONTROL_DT = 0.02        # 50 Hz control loop (20 ms)

# ==============================================================================
# 2. 48-DIMENSIONAL OBSERVATION VECTOR BUILDER
# ==============================================================================
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

class JaguarObservationBuilder:
    def __init__(self):
        self.last_action = torch.zeros((1, 12), dtype=torch.float32)

    def build_observation(self, lin_vel, ang_vel, quat, cmd, joint_pos, joint_vel):
        """
        Builds exact 48D Observation Vector for DreamWaQ Actor:
        [0:3]   Linear Velocity (base frame)
        [3:6]   Angular Velocity (base frame)
        [6:9]   Projected Gravity Vector (R^T * [0, 0, -1] -> [0, 0, -1] when upright)
        [9:12]  Velocity Commands [vx, vy, wz]
        [12:24] Relative Joint Positions (joint_pos - default_pos)
        [24:36] Joint Velocities
        [36:48] Last Action
        """
        qx, qy, qz, qw = quat
        # Gravity projection in body frame (Isaac Lab / Isaac Gym standard: R^T * [0, 0, -1])
        gx = -2.0 * (qx * qz - qw * qy)
        gy = -2.0 * (qy * qz + qw * qx)
        gz = -(1.0 - 2.0 * (qx * qx + qy * qy))

        rel_joint_pos = joint_pos - DEFAULT_JOINT_POS

        obs_vec = np.concatenate([
            lin_vel,                     # 3D: vx, vy, vz
            ang_vel,                     # 3D: wx, wy, wz
            [gx, gy, gz],                # 3D: projected gravity (nominal [0, 0, -1])
            cmd,                         # 3D: vx_cmd, vy_cmd, wz_cmd
            rel_joint_pos,               # 12D: q - q0
            joint_vel,                   # 12D: q_dot
            self.last_action[0].numpy(), # 12D: a_{t-1}
        ], axis=0).astype(np.float32)

        return torch.from_numpy(obs_vec).unsqueeze(0)

    def update_last_action(self, action_tensor):
        self.last_action = action_tensor.clone()

# ==============================================================================
# 3. ROS 2 CONTROLLER NODE
# ==============================================================================
from std_msgs.msg import Bool

class NXPJaguarControllerNode(Node):
    def __init__(self):
        super().__init__("nxp_jaguar_controller")

        # Declare parameters
        self.declare_parameter("policy_path", "")
        policy_param = self.get_parameter("policy_path").get_parameter_value().string_value
        if not policy_param:
            policy_param = os.path.abspath(os.path.join(os.path.dirname(__file__), "../models/policy.pt"))

        self.get_logger().info(f"Loading TorchScript Policy from: {policy_param}")
        self.policy = torch.jit.load(policy_param, map_location="cpu")
        self.policy.eval()

        self.obs_builder = JaguarObservationBuilder()
        self.state_lock = threading.Lock()

        # State Variables
        self.joint_pos = SIT_JOINT_POS.copy()
        self.joint_vel = np.zeros(12, dtype=np.float32)
        self.body_quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
        self.body_ang_vel = np.zeros(3, dtype=np.float32)
        self.body_lin_vel = np.zeros(3, dtype=np.float32)
        self.cmd_vel = np.zeros(3, dtype=np.float32)

        self.state = "STANDBY"   # States: STANDBY (Sit) -> STANDUP -> WALK -> SITDOWN
        self.imu_received = False
        self.joints_received = False

        # Transition interpolation variables
        self.transition_start_pos = SIT_JOINT_POS.copy()
        self.transition_start_time = 0.0
        self.transition_duration = 2.0  # seconds

        # QoS Profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Subscribers
        self.create_subscription(Imu, "/imu/data", self._imu_cb, sensor_qos)
        self.create_subscription(Imu, "/Imu_data", self._imu_cb, sensor_qos)
        self.create_subscription(JointState, "/joint_states", self._joint_state_cb, 10)
        self.create_subscription(Twist, "/cmd_vel", self._cmd_vel_cb, 10)
        self.create_subscription(Joy, "/joy", self._joy_cb, 10)
        self.create_subscription(Bool, "/jaguar/emergency_stop", self._estop_cb, 10)

        # Publishers
        self.joint_cmd_pub = self.create_publisher(JointState, "/joint_commands", 10)
        self.debug_pub = self.create_publisher(Float32MultiArray, "/jaguar/state_debug", 10)
        self.status_pub = self.create_publisher(String, "/jaguar/status", 10)

        # Performance & Frequency Watchdog variables
        self.last_loop_time = time.perf_counter()
        self.actual_freq = 50.0
        self.compute_latency_ms = 0.0
        self.lag_counter = 0
        self.last_warn_time = 0.0
        self.dt_history = []

        # 50 Hz Control Timer Loop (20 ms dt)
        self.timer = self.create_timer(CONTROL_DT, self._control_loop)
        self.get_logger().info("NXP Jaguar ROS 2 Controller Initialized. State: STANDBY")

    def _estop_cb(self, msg: Bool):
        if msg.data:
            with self.state_lock:
                self.state = "STANDBY"
                self.cmd_vel[:] = 0.0
            self.get_logger().warn("🚨 EMERGENCY STOP RECEIVED via topic! Resetting state to STANDBY (DUDUK).")

    def _imu_cb(self, msg: Imu):
        with self.state_lock:
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

            self.body_quat = np.array([body_qx, body_qy, body_qz, body_qw], dtype=np.float32)
            self.body_ang_vel = np.array([
                msg.angular_velocity.x,
                -msg.angular_velocity.y,
                -msg.angular_velocity.z,
            ], dtype=np.float32)
            self.imu_received = True

    def _cmd_vel_cb(self, msg: Twist):
        with self.state_lock:
            self.cmd_vel[0] = float(np.clip(msg.linear.x, -1.0, 1.5))
            self.cmd_vel[1] = float(np.clip(msg.linear.y, -0.6, 0.6))
            self.cmd_vel[2] = float(np.clip(msg.angular.z, -1.5, 1.5))

    def _joint_state_cb(self, msg: JointState):
        with self.state_lock:
            # Map input joint states from message to Isaac Lab joint order
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
                # If unnamed array in ROS order (BL, BR, FL, FR), remap to Isaac order
                for i in range(12):
                    ros_idx = ROS_TO_ISAAC[i]
                    self.joint_pos[i] = msg.position[ros_idx]
                    if len(msg.velocity) > ros_idx:
                        self.joint_vel[i] = msg.velocity[ros_idx]
            self.joints_received = True

    def _joy_cb(self, msg: Joy):
        now = self.get_clock().now().nanoseconds / 1e9
        if len(msg.buttons) > 1:
            # Button 0 (X / Cross / Key '2'): Stand Up (Berdiri)
            if msg.buttons[0] == 1 and self.state in ["STANDBY", "SITDOWN"]:
                self.state = "STANDUP"
                self.transition_start_time = now
                with self.state_lock:
                    self.transition_start_pos = self.joint_pos.copy()
                    self.cmd_vel[:] = 0.0
                self.get_logger().info("State Transition -> STANDUP (Berdiri perlahan)")
            # Button 1 (Circle / B / Key '3'): Start RL Walking (Jalan RL)
            elif msg.buttons[1] == 1 and self.state in ["STANDUP", "STANDBY", "SITDOWN"]:
                self.state = "WALK"
                self.get_logger().info("State Transition -> WALK (RL Policy Active)")
            # Button 2 (Square / X / Key '1'): Smooth Sit Down (Duduk perlahan)
            elif len(msg.buttons) > 2 and msg.buttons[2] == 1:
                if self.state in ["STANDUP", "WALK"]:
                    self.state = "SITDOWN"
                    self.transition_start_time = now
                    with self.state_lock:
                        self.transition_start_pos = self.joint_pos.copy()
                        self.cmd_vel[:] = 0.0
                    self.get_logger().info("State Transition -> SITDOWN (Duduk perlahan)")
                else:
                    self.state = "STANDBY"
                    with self.state_lock:
                        self.cmd_vel[:] = 0.0
                    self.get_logger().info("State -> STANDBY (Duduk / Folded)")

        # Joystick axes mapped to command velocities
        if len(msg.axes) >= 2:
            with self.state_lock:
                self.cmd_vel[0] = msg.axes[1] * 1.0   # Left stick vertical: vx
                self.cmd_vel[1] = msg.axes[0] * 0.5   # Left stick horizontal: vy
                # Right stick horizontal: wz (axis 3 for Xbox/PS4, or axis 2 fallback)
                if len(msg.axes) > 3:
                    self.cmd_vel[2] = msg.axes[3] * 1.2
                elif len(msg.axes) >= 3:
                    self.cmd_vel[2] = msg.axes[2] * 1.2

    def _control_loop(self):
        t_start = time.perf_counter()
        dt = t_start - self.last_loop_time
        self.last_loop_time = t_start

        # Track rolling frequency over last 25 cycles
        if 0.001 < dt < 1.0:
            self.dt_history.append(dt)
            if len(self.dt_history) > 25:
                self.dt_history.pop(0)
            self.actual_freq = 1.0 / (sum(self.dt_history) / len(self.dt_history))

        if not self.imu_received:
            return

        with self.state_lock:
            pos = self.joint_pos.copy()
            vel = self.joint_vel.copy()
            quat = self.body_quat.copy()
            ang_v = self.body_ang_vel.copy()
            lin_v = self.body_lin_vel.copy()
            cmd = self.cmd_vel.copy()

        # Tilt Safety Protection: Emergency Cutoff if robot tilts > 60 degrees (gz > -0.5 when upright is -1.0)
        gz_body = -(1.0 - 2.0 * (quat[0]**2 + quat[1]**2))
        if gz_body > -0.5:
            self.get_logger().error(f"EMERGENCY TILT DETECTED (gz={gz_body:.2f} > -0.5, tilt > 60 deg)! Resetting to STANDBY.")
            self.state = "STANDBY"

        now = self.get_clock().now().nanoseconds / 1e9

        if self.state == "STANDBY":
            target_pos = SIT_JOINT_POS.copy()
        elif self.state == "STANDUP":
            elapsed = now - self.transition_start_time
            alpha = float(np.clip(elapsed / self.transition_duration, 0.0, 1.0))
            # Smooth S-curve interpolation
            smooth_alpha = 0.5 * (1.0 - math.cos(math.pi * alpha))
            target_pos = (1.0 - smooth_alpha) * self.transition_start_pos + smooth_alpha * DEFAULT_JOINT_POS
            if alpha >= 1.0:
                target_pos = DEFAULT_JOINT_POS.copy()
        elif self.state == "SITDOWN":
            elapsed = now - self.transition_start_time
            alpha = float(np.clip(elapsed / self.transition_duration, 0.0, 1.0))
            smooth_alpha = 0.5 * (1.0 - math.cos(math.pi * alpha))
            target_pos = (1.0 - smooth_alpha) * self.transition_start_pos + smooth_alpha * SIT_JOINT_POS
            if alpha >= 1.0:
                self.state = "STANDBY"
                target_pos = SIT_JOINT_POS.copy()
                self.get_logger().info("SITDOWN Complete -> State: STANDBY")
        elif self.state == "WALK":
            # 1. Build 48-Dimensional Observation Vector
            obs = self.obs_builder.build_observation(lin_v, ang_v, quat, cmd, pos, vel)

            # 2. Neural Network Forward Inference (JIT Model)
            with torch.no_grad():
                actions = self.policy(obs)
                self.obs_builder.update_last_action(actions)

            raw_action = actions.squeeze(0).numpy()
            target_pos = DEFAULT_JOINT_POS + ACTION_SCALE * raw_action

            # 3. Publish Debug 48D State Vector
            debug_msg = Float32MultiArray()
            debug_msg.data = obs.squeeze(0).numpy().tolist()
            self.debug_pub.publish(debug_msg)

        # Publish Joint Commands to CAN Motor Node
        cmd_msg = JointState()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.name = ISAAC_JOINT_NAMES
        cmd_msg.position = target_pos.tolist()
        cmd_msg.velocity = [0.0] * 12
        cmd_msg.effort = [0.0] * 12
        self.joint_cmd_pub.publish(cmd_msg)

        t_end = time.perf_counter()
        self.compute_latency_ms = (t_end - t_start) * 1000.0
        now_sec = self.get_clock().now().nanoseconds / 1e9

        # Frequency & Compute Latency Watchdog / Warnings
        if self.compute_latency_ms > 20.0 or self.actual_freq < 45.0:
            self.lag_counter += 1
            if now_sec - self.last_warn_time > 1.0:
                self.get_logger().warn(
                    f"⚠️ [TIMING WARNING] Controller Compute Lag! "
                    f"Latency: {self.compute_latency_ms:.2f} ms (Budget: 20.0 ms) | "
                    f"Actual Freq: {self.actual_freq:.1f} Hz (Target: 50.0 Hz)"
                )
                self.last_warn_time = now_sec

            # Critical Safety Fallback: If severe lag occurs in WALK mode (compute > 45ms or freq < 25Hz for 5 consecutive loops)
            if self.state == "WALK" and (self.compute_latency_ms > 45.0 or self.actual_freq < 25.0):
                if self.lag_counter >= 5:
                    self.get_logger().error(
                        f"🚨 [CRITICAL WATCHDOG] Persistent compute lag ({self.compute_latency_ms:.1f} ms, {self.actual_freq:.1f} Hz)! "
                        f"Safety fallback triggered -> Resetting to STANDUP."
                    )
                    self.state = "STANDUP"
                    self.transition_start_time = now_sec
                    with self.state_lock:
                        self.transition_start_pos = self.joint_pos.copy()
                        self.cmd_vel[:] = 0.0
                    self.lag_counter = 0
        else:
            self.lag_counter = max(0, self.lag_counter - 1)

        # Publish Status String
        status_msg = String()
        status_msg.data = f"State: {self.state} | Cmd: [{cmd[0]:.2f}, {cmd[1]:.2f}, {cmd[2]:.2f}] | Freq: {self.actual_freq:.1f} Hz | Latency: {self.compute_latency_ms:.1f} ms"
        self.status_pub.publish(status_msg)



def main(args=None):
    rclpy.init(args=args)
    node = NXPJaguarControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down NXP Jaguar Controller...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
