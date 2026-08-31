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

# Default Standing Pose synchronized with Isaac Lab NXP Jaguar (25 cm Stance)
DEFAULT_JOINT_POS = np.array([
    0.0,   0.0,   0.0,   0.0,    # Rolls (Fr, Fl, Br, Bl)
   -1.55, -1.55, -1.45, -1.45,   # Hip Pitches (Fr, Fl, Br, Bl)
    1.42,  1.42,  1.35,  1.35,   # Knees (Fr, Fl, Br, Bl)
], dtype=np.float32)

ACTION_SCALE = 0.25      # Policy action scaling factor
CONTROL_DT = 0.02        # 50 Hz control loop (20 ms)

# Gain Scheduling Constants (Coxa/Roll vs Hip/Knee)
# Coxa/Roll (4 joints): Stiffness 18.0, Damping 2.0
# Hip & Knee (8 joints): Stiffness 25.0, Damping 1.5
RL_KP_ROLL = 18.0
RL_KD_ROLL = 2.0
RL_KP_PITCH = 25.0
RL_KD_PITCH = 1.5

TRANSITION_KP_ROLL = 18.0
TRANSITION_KD_ROLL = 2.0
TRANSITION_KP_PITCH = 25.0
TRANSITION_KD_PITCH = 1.5

# Isaac order: [0..3 Rolls, 4..7 Hips, 8..11 Knees]
DEFAULT_TRANSITION_KP = [TRANSITION_KP_ROLL] * 4 + [TRANSITION_KP_PITCH] * 8
DEFAULT_TRANSITION_KD = [TRANSITION_KD_ROLL] * 4 + [TRANSITION_KD_PITCH] * 8

DEFAULT_RL_KP = [RL_KP_ROLL] * 4 + [RL_KP_PITCH] * 8
DEFAULT_RL_KD = [RL_KD_ROLL] * 4 + [RL_KD_PITCH] * 8

MAX_HOMING_VEL = 0.35    # rad/s (ultra-smooth continuous rate limit for startup homing)

# ==============================================================================
# 2. 45-DIMENSIONAL TEMPORAL OBSERVATION BUILDER (DREAMWAQ CENET)
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
    def __init__(self, history_len: int = 5):
        self.history_len = history_len
        self.obs_dim = 45
        self.last_action = np.zeros(12, dtype=np.float32)
        # Temporal history ring buffer: shape (1, 5, 45)
        self.history_buf = np.zeros((1, history_len, self.obs_dim), dtype=np.float32)

    def reset_history(self, initial_obs_45d: np.ndarray):
        for i in range(self.history_len):
            self.history_buf[0, i, :] = initial_obs_45d
        self.last_action[:] = 0.0

    def build_step_observation(self, ang_vel, quat, cmd, joint_pos, joint_vel) -> np.ndarray:
        """
        Builds 45D proprioceptive observation vector:
        [0:3]   Angular Velocity (base frame gyro)
        [3:6]   Projected Gravity Vector
        [6:9]   Velocity Commands [vx, vy, wz]
        [9:21]  Relative Joint Positions (joint_pos - default_pos)
        [21:33] Joint Velocities
        [33:45] Last Action
        """
        qx, qy, qz, qw = quat
        # Gravity projection in body frame (R^T * [0, 0, -1])
        gx = -2.0 * (qx * qz - qw * qy)
        gy = -2.0 * (qy * qz + qw * qx)
        gz = -(1.0 - 2.0 * (qx * qx + qy * qy))

        rel_joint_pos = joint_pos - DEFAULT_JOINT_POS

        obs_45d = np.concatenate([
            ang_vel,                     # 3D: wx, wy, wz
            [gx, gy, gz],                # 3D: projected gravity
            cmd,                         # 3D: vx_cmd, vy_cmd, wz_cmd
            rel_joint_pos,               # 12D: q - q0
            joint_vel,                   # 12D: q_dot
            self.last_action,            # 12D: a_{t-1}
        ], axis=0).astype(np.float32)

        return obs_45d

    def update_and_get_history(self, obs_45d: np.ndarray) -> torch.Tensor:
        # Shift history left and insert newest observation at the end
        self.history_buf = np.roll(self.history_buf, shift=-1, axis=1)
        self.history_buf[0, -1, :] = obs_45d
        return torch.from_numpy(self.history_buf).float()

    def update_last_action(self, action_np: np.ndarray):
        self.last_action = action_np.copy()

# ==============================================================================
# 3. ROS 2 CONTROLLER NODE
# ==============================================================================
from std_msgs.msg import Bool

class NXPJaguarControllerNode(Node):
    def __init__(self):
        super().__init__("nxp_jaguar_controller")

        # Declare parameters
        self.declare_parameter("policy_path", "")
        self.declare_parameter("torque_limit", 14.0)
        self.declare_parameter("shutdown_duration", 3.0)
        self.declare_parameter("shutdown_settle_delay", 0.5)
        policy_param = self.get_parameter("policy_path").get_parameter_value().string_value
        if not policy_param:
            policy_param = os.path.abspath(os.path.join(os.path.dirname(__file__), "../models/policy.pt"))

        self.torque_limit = float(self.get_parameter("torque_limit").value)
        self.shutdown_duration = float(self.get_parameter("shutdown_duration").value)
        self.shutdown_settle_delay = float(self.get_parameter("shutdown_settle_delay").value)
        self.sitdown_settle_delay = 0.5
        self.torque_overload_cycles = 5  # 100 ms debounce at 50 Hz

        # Action EMA Low-Pass Filter (removes high-frequency jitter/chatter)
        self.declare_parameter("action_ema_alpha", 0.7)
        self.action_ema_alpha = float(self.get_parameter("action_ema_alpha").value)
        self.filtered_action = np.zeros(12, dtype=np.float32)

        self.get_logger().info(f"Loading TorchScript Policy from: {policy_param}")
        self.policy = torch.jit.load(policy_param, map_location="cpu")
        self.policy.eval()

        self.obs_builder = JaguarObservationBuilder()
        self.state_lock = threading.Lock()

        # State Variables
        self.joint_pos = SIT_JOINT_POS.copy()
        self.joint_vel = np.zeros(12, dtype=np.float32)
        self.joint_tau = np.zeros(12, dtype=np.float32)
        self.body_quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
        self.body_ang_vel = np.zeros(3, dtype=np.float32)
        self.body_lin_vel = np.zeros(3, dtype=np.float32)
        self.cmd_vel = np.zeros(3, dtype=np.float32)

        self.state = "STANDBY"   # States: STANDBY (Passive Zero Torque) -> STANDUP -> WALK -> SITDOWN -> SAFE_SHUTDOWN -> DISABLED
        self.imu_received = False
        self.joints_received = False
        self.overtorque_counter = 0

        # Transition interpolation variables
        self.transition_start_pos = SIT_JOINT_POS.copy()
        self.transition_target_pos = SIT_JOINT_POS.copy()
        self.transition_start_time = 0.0
        self.transition_duration = 4.0   # seconds for stand/sit transitions (matching 4.0s smooth S-curve)

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
        self.create_subscription(Bool, "/jaguar/safe_stop", self._safe_stop_cb, 10)
        self.create_subscription(Bool, "/jaguar/emergency_stop", self._estop_cb, 10)

        # Publishers
        self.joint_cmd_pub = self.create_publisher(JointState, "/joint_commands", 10)
        self.estop_pub = self.create_publisher(Bool, "/jaguar/emergency_stop", 10)
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
        self.get_logger().info("NXP Jaguar ROS 2 Controller Initialized. State: STANDBY (Motors Passive, Zero Torque)")

    def _trigger_safe_shutdown(self, now: float, reason: str):
        if self.state in ["SAFE_SHUTDOWN", "DISABLED"]:
            return
        self.get_logger().warn(
            f"[FAILSAFE] {reason}. Returning to zero in {self.shutdown_duration:.1f}s, then holding for {self.shutdown_settle_delay:.1f}s before motor cutoff."
        )
        self.state = "SAFE_SHUTDOWN"
        self.transition_start_time = now
        with self.state_lock:
            self.transition_start_pos = self.joint_pos.copy()
            diff = (SIT_JOINT_POS - self.transition_start_pos + np.pi) % (2 * np.pi) - np.pi
            self.transition_target_pos = self.transition_start_pos + diff
            self.cmd_vel[:] = 0.0

    def _safe_stop_cb(self, msg: Bool):
        if msg.data:
            now = self.get_clock().now().nanoseconds / 1e9
            self._trigger_safe_shutdown(now, "Safe stop requested via topic")

    def _estop_cb(self, msg: Bool):
        if msg.data and self.state not in ["SAFE_SHUTDOWN", "DISABLED", "STANDBY"]:
            now = self.get_clock().now().nanoseconds / 1e9
            self._trigger_safe_shutdown(now, "Emergency stop signal received")

    def _imu_cb(self, msg: Imu):
        with self.state_lock:
            # Reorient IMU frame if mounted upside-down (Roll 180 deg)
            qx = msg.orientation.x
            qy = msg.orientation.y
            qz = msg.orientation.z
            qw = msg.orientation.w

            body_qx = qw
            body_qy = qz
            body_qz = -qy
            body_qw = -qx

            self.body_quat[0] = body_qx
            self.body_quat[1] = body_qy
            self.body_quat[2] = body_qz
            self.body_quat[3] = body_qw

            self.body_ang_vel[0] = msg.angular_velocity.x
            self.body_ang_vel[1] = -msg.angular_velocity.y
            self.body_ang_vel[2] = -msg.angular_velocity.z

            self.imu_received = True

    def _cmd_vel_cb(self, msg: Twist):
        with self.state_lock:
            cmd_deadzone = 0.04
            vx = float(np.clip(msg.linear.x, -0.8, 0.8))
            vy = float(np.clip(msg.linear.y, -0.5, 0.5))
            wz = float(np.clip(msg.angular.z, -0.8, 0.8))
            self.cmd_vel[0] = vx if abs(vx) > cmd_deadzone else 0.0
            self.cmd_vel[1] = vy if abs(vy) > cmd_deadzone else 0.0
            self.cmd_vel[2] = wz if abs(wz) > cmd_deadzone else 0.0

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
                        if len(msg.effort) > idx:
                            self.joint_tau[isaac_idx] = msg.effort[idx]
                        matched = True
            if not matched and len(msg.position) == 12:
                # If unnamed array in ROS order (BL, BR, FL, FR), remap to Isaac order
                for i in range(12):
                    ros_idx = ROS_TO_ISAAC[i]
                    self.joint_pos[i] = msg.position[ros_idx]
                    if len(msg.velocity) > ros_idx:
                        self.joint_vel[i] = msg.velocity[ros_idx]
                    if len(msg.effort) > ros_idx:
                        self.joint_tau[i] = msg.effort[ros_idx]
            self.joints_received = True

    def _joy_cb(self, msg: Joy):
        now = self.get_clock().now().nanoseconds / 1e9
        if len(msg.buttons) > 1:
            # Button 0 (X / Cross / Key '2'): Stand Up (Berdiri)
            if msg.buttons[0] == 1 and self.state in ["STARTUP_SIT", "STANDBY", "SITDOWN", "SIT_HOLD", "DISABLED"]:
                self.state = "STANDUP"
                self.transition_start_time = now
                with self.state_lock:
                    self.transition_start_pos = self.joint_pos.copy()
                    diff = (DEFAULT_JOINT_POS - self.transition_start_pos + np.pi) % (2 * np.pi) - np.pi
                    self.transition_target_pos = self.transition_start_pos + diff
                    self.cmd_vel[:] = 0.0
                    self.filtered_action[:] = 0.0
                self.get_logger().info(f"[CONTROLLER] State transition -> STANDUP ({self.transition_duration:.1f}s smooth S-curve)")
            # Button 1 (Circle / B / Key '3'): Start RL Walking (Jalan RL)
            elif msg.buttons[1] == 1 and self.state in ["STARTUP_SIT", "STANDUP", "STAND_HOLD", "STANDBY", "SITDOWN", "SIT_HOLD"]:
                self.state = "WALK"
                with self.state_lock:
                    self.filtered_action[:] = 0.0
                    init_obs = self.obs_builder.build_step_observation(self.body_ang_vel, self.body_quat, self.cmd_vel, self.joint_pos, self.joint_vel)
                    self.obs_builder.reset_history(init_obs)
                self.get_logger().info(
                    f"[CONTROLLER] State transition -> WALK (DreamWaQ CENet | Gains: Coxa[Kp={RL_KP_ROLL}, Kd={RL_KD_ROLL}], Leg[Kp={RL_KP_PITCH}, Kd={RL_KD_PITCH}] | Action EMA alpha={self.action_ema_alpha})"
                )
            # Button 2 (Square / X / Key '1'): Smooth Sit Down (Duduk perlahan)
            elif len(msg.buttons) > 2 and msg.buttons[2] == 1:
                if self.state != "SIT_HOLD":
                    self.state = "SITDOWN"
                    self.transition_start_time = now
                    with self.state_lock:
                        self.transition_start_pos = self.joint_pos.copy()
                        diff = (SIT_JOINT_POS - self.transition_start_pos + np.pi) % (2 * np.pi) - np.pi
                        self.transition_target_pos = self.transition_start_pos + diff
                        self.cmd_vel[:] = 0.0
                        self.filtered_action[:] = 0.0
                    self.get_logger().info(f"[CONTROLLER] State transition -> SITDOWN ({self.transition_duration:.1f}s smooth S-curve)")

        # Joystick axes with 15% Deadzone (Eliminates stick drift when released, max 0.8 m/s, 0.5 m/s, 0.8 rad/s)
        joy_deadzone = 0.15
        if len(msg.axes) >= 2:
            with self.state_lock:
                raw_vx = msg.axes[1]
                raw_vy = msg.axes[0]
                raw_wz = msg.axes[3] if len(msg.axes) > 3 else (msg.axes[2] if len(msg.axes) >= 3 else 0.0)

                self.cmd_vel[0] = (raw_vx * 0.8) if abs(raw_vx) > joy_deadzone else 0.0
                self.cmd_vel[1] = (raw_vy * 0.5) if abs(raw_vy) > joy_deadzone else 0.0
                self.cmd_vel[2] = (raw_wz * 0.8) if abs(raw_wz) > joy_deadzone else 0.0

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

        if not self.imu_received or not self.joints_received:
            return

        with self.state_lock:
            pos = self.joint_pos.copy()
            vel = self.joint_vel.copy()
            tau = self.joint_tau.copy()
            quat = self.body_quat.copy()
            ang_v = self.body_ang_vel.copy()
            lin_v = self.body_lin_vel.copy()
            cmd = self.cmd_vel.copy()

        now = self.get_clock().now().nanoseconds / 1e9

        # Failsafe 1: Over-Torque Protection (Continuous overload > threshold for >100ms in active states)
        max_tau = float(np.max(np.abs(tau)))
        if max_tau > self.torque_limit and self.state in ["STANDUP", "WALK", "SITDOWN"]:
            self.overtorque_counter += 1
            if self.overtorque_counter >= self.torque_overload_cycles:
                joint_idx = int(np.argmax(np.abs(tau)))
                joint_name = ISAAC_JOINT_NAMES[joint_idx]
                self._trigger_safe_shutdown(
                    now, f"Over-torque on {joint_name} ({max_tau:.2f} Nm > {self.torque_limit:.2f} Nm)"
                )
        else:
            self.overtorque_counter = max(0, self.overtorque_counter - 1)

        # Failsafe 2: Tilt Safety Protection (Emergency sit if tilt > 60 deg, gz > -0.5 in active states)
        gz_body = -(1.0 - 2.0 * (quat[0]**2 + quat[1]**2))
        if gz_body > -0.5 and self.state in ["STANDUP", "WALK"]:
            self._trigger_safe_shutdown(now, f"Critical tilt detected (gz={gz_body:.2f} > -0.5, tilt > 60 deg)")

        target_pos = None
        target_vel = np.zeros(12, dtype=np.float32)
        cmd_kp = DEFAULT_TRANSITION_KP[:]
        cmd_kd = DEFAULT_TRANSITION_KD[:]

        if self.state in ["STANDBY", "DISABLED"]:
            # 100% passive zero-torque sensing mode (No stiffness, Motors limp, Zero Torque)
            target_pos = SIT_JOINT_POS.copy()
            target_vel = np.zeros(12, dtype=np.float32)
            cmd_kp = [0.0] * 12
            cmd_kd = [0.0] * 12
        elif self.state == "STANDUP":
            elapsed = now - self.transition_start_time
            alpha = float(np.clip(elapsed / self.transition_duration, 0.0, 1.0))
            # Smooth S-curve interpolation from measured starting posture
            smooth_alpha = 0.5 * (1.0 - math.cos(math.pi * alpha))
            diff = self.transition_target_pos - self.transition_start_pos
            target_pos = self.transition_start_pos + smooth_alpha * diff
            target_vel = (math.pi / (2.0 * self.transition_duration)) * math.sin(math.pi * alpha) * diff
            cmd_kp = DEFAULT_TRANSITION_KP[:]
            cmd_kd = DEFAULT_TRANSITION_KD[:]
            if alpha >= 1.0:
                self.state = "STAND_HOLD"
                target_pos = DEFAULT_JOINT_POS.copy()
                target_vel = np.zeros(12, dtype=np.float32)
                self.get_logger().info("[CONTROLLER] STANDUP complete -> Holding standing pose firmly (Power ON)")
        elif self.state == "STAND_HOLD":
            # Actively hold standing pose with power ON
            target_pos = DEFAULT_JOINT_POS.copy()
            target_vel = np.zeros(12, dtype=np.float32)
            cmd_kp = DEFAULT_TRANSITION_KP[:]
            cmd_kd = DEFAULT_TRANSITION_KD[:]
        elif self.state == "SITDOWN":
            elapsed = now - self.transition_start_time
            alpha = float(np.clip(elapsed / self.transition_duration, 0.0, 1.0))
            smooth_alpha = 0.5 * (1.0 - math.cos(math.pi * alpha))
            diff = self.transition_target_pos - self.transition_start_pos
            target_pos = self.transition_start_pos + smooth_alpha * diff
            target_vel = (math.pi / (2.0 * self.transition_duration)) * math.sin(math.pi * alpha) * diff
            cmd_kp = DEFAULT_TRANSITION_KP[:]
            cmd_kd = DEFAULT_TRANSITION_KD[:]
            if alpha >= 1.0:
                target_pos = SIT_JOINT_POS.copy()
                target_vel = np.zeros(12, dtype=np.float32)
                if elapsed >= (self.transition_duration + self.sitdown_settle_delay):
                    self.state = "STANDBY"
                    cmd_kp = [0.0] * 12
                    cmd_kd = [0.0] * 12
                    self.get_logger().info("[CONTROLLER] SITDOWN complete -> Motors relaxed to STANDBY (Passive Zero-Torque)")
        elif self.state == "SAFE_SHUTDOWN":
            elapsed = now - self.transition_start_time
            alpha = float(np.clip(elapsed / self.shutdown_duration, 0.0, 1.0))
            smooth_alpha = 0.5 * (1.0 - math.cos(math.pi * alpha))
            diff = self.transition_target_pos - self.transition_start_pos
            target_pos = self.transition_start_pos + smooth_alpha * diff
            target_vel = (math.pi / (2.0 * self.shutdown_duration)) * math.sin(math.pi * alpha) * diff
            cmd_kp = DEFAULT_TRANSITION_KP[:]
            cmd_kd = DEFAULT_TRANSITION_KD[:]
            if alpha >= 1.0:
                target_pos = SIT_JOINT_POS.copy()
                target_vel = np.zeros(12, dtype=np.float32)
                # Settle delay: actively hold 0.0 rad for settle delay before cutting motor torque
                if elapsed >= (self.shutdown_duration + self.shutdown_settle_delay):
                    self.state = "STANDBY"
                    cmd_kp = [0.0] * 12
                    cmd_kd = [0.0] * 12
                    estop_msg = Bool()
                    estop_msg.data = True
                    self.estop_pub.publish(estop_msg)
                    self.get_logger().info("[FAILSAFE] Robot in sit pose. Motors relaxed to Passive Zero-Torque.")
        elif self.state == "WALK":
            # 1. Build 45-Dimensional Step Observation & Update 5-Step History Buffer (1, 5, 45)
            obs_45d = self.obs_builder.build_step_observation(ang_v, quat, cmd, pos, vel)
            history_tensor = self.obs_builder.update_and_get_history(obs_45d)

            # 2. Neural Network Forward Inference (Fused CENet + Actor JIT Model)
            with torch.no_grad():
                actions = self.policy(history_tensor)

            raw_action = actions.squeeze(0).cpu().numpy()
            self.obs_builder.update_last_action(raw_action)

            # Apply Single Action EMA Low-Pass Filter on Policy Actions (alpha=0.6~0.7)
            self.filtered_action = (
                self.action_ema_alpha * self.filtered_action + (1.0 - self.action_ema_alpha) * raw_action
            )

            target_pos = DEFAULT_JOINT_POS + ACTION_SCALE * self.filtered_action
            target_vel = np.zeros(12, dtype=np.float32)

            # High impedance tracking gains specifically for RL (Coxa: Kp=18, Kd=2.0 | Leg: Kp=25, Kd=1.5)
            cmd_kp = DEFAULT_RL_KP[:]
            cmd_kd = DEFAULT_RL_KD[:]

            # 3. Publish Debug State Vector
            debug_msg = Float32MultiArray()
            debug_msg.data = obs_45d.tolist()
            self.debug_pub.publish(debug_msg)

        # Publish Joint Commands to CAN Motor Node only during active states
        if target_pos is not None:
            cmd_msg = JointState()
            cmd_msg.header.stamp = self.get_clock().now().to_msg()
            cmd_msg.name = ISAAC_JOINT_NAMES
            cmd_msg.position = target_pos.tolist()
            cmd_msg.velocity = target_vel.tolist()
            cmd_msg.effort = (cmd_kp + cmd_kd)
            self.joint_cmd_pub.publish(cmd_msg)

        t_end = time.perf_counter()
        self.compute_latency_ms = (t_end - t_start) * 1000.0
        now_sec = self.get_clock().now().nanoseconds / 1e9

        # Frequency & Compute Latency Watchdog / Warnings
        if self.compute_latency_ms > 20.0 or self.actual_freq < 45.0:
            self.lag_counter += 1
            if now_sec - self.last_warn_time > 1.0:
                self.get_logger().warn(
                    f"[TIMING] Compute lag: {self.compute_latency_ms:.2f} ms | "
                    f"Frequency: {self.actual_freq:.1f} Hz (Target: 50.0 Hz)"
                )
                self.last_warn_time = now_sec

            # Critical Safety Fallback: If severe lag occurs in WALK mode (compute > 45ms or freq < 25Hz for 5 consecutive loops)
            if self.state == "WALK" and (self.compute_latency_ms > 45.0 or self.actual_freq < 25.0):
                if self.lag_counter >= 5:
                    self.get_logger().error(
                        f"[WATCHDOG] Persistent compute lag ({self.compute_latency_ms:.1f} ms, {self.actual_freq:.1f} Hz). "
                        f"Triggering safe shutdown."
                    )
                    self._trigger_safe_shutdown(now_sec, "Persistent compute lag")
                    self.lag_counter = 0
        else:
            self.lag_counter = max(0, self.lag_counter - 1)

        # Publish Status String
        status_msg = String()
        status_msg.data = f"State: {self.state} | Cmd: [{cmd[0]:.2f}, {cmd[1]:.2f}, {cmd[2]:.2f}] | Freq: {self.actual_freq:.1f} Hz | Latency: {self.compute_latency_ms:.1f} ms | MaxTau: {max_tau:.1f} Nm"
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
