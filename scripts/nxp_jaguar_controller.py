#!/usr/bin/env python3
"""
NXP Jaguar Sim-to-Real Controller Node for ROS 2.
Deploys Isaac Lab 3.0 TorchScript Policy (DreamWaQ) to RobStride RS00 Hardware.
"""

import os
import math
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

# Default Standing Pose (Matching NXP_JAGUAR_CFG in Isaac Lab)
DEFAULT_JOINT_POS = np.array([
    0.0,  0.0,  0.0,  0.0,   # Rolls
   -1.5, -1.5, -1.5, -1.5,   # Hip Pitches
    1.5,  1.5,  1.5,  1.5,   # Knees
], dtype=np.float32)

ACTION_SCALE = 0.25      # Policy action scaling factor
CONTROL_DT = 0.02        # 50 Hz control loop (20 ms)

# ==============================================================================
# 2. 48-DIMENSIONAL OBSERVATION VECTOR BUILDER
# ==============================================================================
class JaguarObservationBuilder:
    def __init__(self):
        self.last_action = torch.zeros((1, 12), dtype=torch.float32)

    def build_observation(self, lin_vel, ang_vel, quat, cmd, joint_pos, joint_vel):
        """
        Builds exact 48D Observation Vector for DreamWaQ Actor:
        [0:3]   Linear Velocity (base frame)
        [3:6]   Angular Velocity (base frame)
        [6:9]   Projected Gravity Vector
        [9:12]  Velocity Commands [vx, vy, wz]
        [12:24] Relative Joint Positions (joint_pos - default_pos)
        [24:36] Joint Velocities
        [36:48] Last Action
        """
        qx, qy, qz, qw = quat
        # Gravity projection in body frame
        gx = 2.0 * (qx * qz - qw * qy)
        gy = 2.0 * (qy * qz + qw * qx)
        gz = 1.0 - 2.0 * (qx * qx + qy * qy)

        rel_joint_pos = joint_pos - DEFAULT_JOINT_POS

        obs_vec = np.concatenate([
            lin_vel,                     # 3D: vx, vy, vz
            ang_vel,                     # 3D: wx, wy, wz
            [gx, gy, gz],                # 3D: projected gravity
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
        self.joint_pos = DEFAULT_JOINT_POS.copy()
        self.joint_vel = np.zeros(12, dtype=np.float32)
        self.body_quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
        self.body_ang_vel = np.zeros(3, dtype=np.float32)
        self.body_lin_vel = np.zeros(3, dtype=np.float32)
        self.cmd_vel = np.zeros(3, dtype=np.float32)

        self.state = "STANDBY"   # States: STANDBY -> STANDUP -> WALK
        self.imu_received = False
        self.joints_received = False

        # QoS Profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Subscribers
        self.create_subscription(Imu, "/imu/data", self._imu_cb, sensor_qos)
        self.create_subscription(JointState, "/joint_states", self._joint_state_cb, 10)
        self.create_subscription(Twist, "/cmd_vel", self._cmd_vel_cb, 10)
        self.create_subscription(Joy, "/joy", self._joy_cb, 10)

        # Publishers
        self.joint_cmd_pub = self.create_publisher(JointState, "/joint_commands", 10)
        self.debug_pub = self.create_publisher(Float32MultiArray, "/jaguar/state_debug", 10)
        self.status_pub = self.create_publisher(String, "/jaguar/status", 10)

        # 50 Hz Control Timer Loop (20 ms dt)
        self.timer = self.create_timer(CONTROL_DT, self._control_loop)
        self.get_logger().info("NXP Jaguar ROS 2 Controller Initialized. State: STANDBY")

    def _imu_cb(self, msg: Imu):
        with self.state_lock:
            self.body_quat = np.array([
                msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
            ], dtype=np.float32)
            self.body_ang_vel = np.array([
                msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
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
            for i, name in enumerate(ISAAC_JOINT_NAMES):
                if name in msg.name:
                    idx = msg.name.index(name)
                    self.joint_pos[i] = msg.position[idx]
                    if len(msg.velocity) > idx:
                        self.joint_vel[i] = msg.velocity[idx]
                elif len(msg.position) == 12:
                    # If unnamed array in ROS order (BL, BR, FL, FR), remap directly
                    ros_idx = ISAAC_TO_ROS[i]
                    self.joint_pos[i] = msg.position[ros_idx]
                    if len(msg.velocity) > ros_idx:
                        self.joint_vel[i] = msg.velocity[ros_idx]
            self.joints_received = True

    def _joy_cb(self, msg: Joy):
        if len(msg.buttons) > 1:
            # Button 0 (X / Cross): Stand Up
            if msg.buttons[0] == 1 and self.state == "STANDBY":
                self.state = "STANDUP"
                self.get_logger().info("State Transition -> STANDUP")
            # Button 1 (Circle / B): Start RL Walking
            elif msg.buttons[1] == 1 and self.state in ["STANDUP", "STANDBY"]:
                self.state = "WALK"
                self.get_logger().info("State Transition -> WALK (Policy Active)")
            # Button 2 (Square / X): Emergency Standby
            elif len(msg.buttons) > 2 and msg.buttons[2] == 1:
                self.state = "STANDBY"
                self.get_logger().warn("E-STOP Pressed -> STANDBY")

        # Joystick axes mapped to command velocities
        if len(msg.axes) >= 3:
            with self.state_lock:
                self.cmd_vel[0] = msg.axes[1] * 1.0   # Left stick vertical: vx
                self.cmd_vel[1] = msg.axes[0] * 0.5   # Left stick horizontal: vy
                self.cmd_vel[2] = msg.axes[2] * 1.2   # Right stick horizontal: wz

    def _control_loop(self):
        if not self.imu_received:
            return

        with self.state_lock:
            pos = self.joint_pos.copy()
            vel = self.joint_vel.copy()
            quat = self.body_quat.copy()
            ang_v = self.body_ang_vel.copy()
            lin_v = self.body_lin_vel.copy()
            cmd = self.cmd_vel.copy()

        # Tilt Safety Protection: Emergency Cutoff if robot tilts > 60 degrees
        rot_matrix_z = 1.0 - 2.0 * (quat[0]**2 + quat[1]**2)
        if rot_matrix_z < 0.5:
            self.get_logger().error("EMERGENCY TILT DETECTED (>60 deg)! Resetting to STANDBY.")
            self.state = "STANDBY"

        if self.state == "STANDBY":
            target_pos = DEFAULT_JOINT_POS.copy()
        elif self.state == "STANDUP":
            target_pos = DEFAULT_JOINT_POS.copy()
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

        # Publish Status String
        status_msg = String()
        status_msg.data = f"State: {self.state} | Cmd: [{cmd[0]:.2f}, {cmd[1]:.2f}, {cmd[2]:.2f}]"
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
