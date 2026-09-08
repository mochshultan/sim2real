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
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
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

import parameters as P

# Remapping Indices: ROS CAN Order (BL, BR, FL, FR) <-> Isaac Lab Order
ROS_TO_ISAAC = [9, 6, 3, 0, 10, 7, 4, 1, 11, 8, 5, 2]
ISAAC_TO_ROS = [3, 7, 11, 2, 6, 10, 1, 5, 9, 0, 4, 8]

# Standby Sitting Joint Angles (0.0 rad)
SIT_JOINT_POS = np.zeros(12, dtype=np.float32)

# Relax Joint Angles (Equal to calibrated motor offsets in Isaac order; raw motor at 0.0 rad)
# Isaac order: [0..3 Rolls (FR, FL, BR, BL), 4..7 Hips (FR, FL, BR, BL), 8..11 Knees (FR, FL, BR, BL)]
RELAX_JOINT_POS = np.array([P.MOTOR_OFFSET_ANGLE[ROS_TO_ISAAC[i]] for i in range(12)], dtype=np.float32)

# Default Standing Pose synchronized with Isaac Lab NXP Jaguar (nxp_jaguar.py)
DEFAULT_JOINT_POS = np.array([
    0.0,   0.0,   0.0,   0.0,    # Rolls (Fr, Fl, Br, Bl)
   -1.40, -1.40, -1.30, -1.30,   # Hip Pitches (Fr, Fl, Br, Bl)
    1.45,  1.45,  1.55,  1.55,   # Knees (Fr, Fl, Br, Bl)
], dtype=np.float32)

# Hard Physical Limits in Isaac Lab Joint Order: 4 Rolls (Fr, Fl, Br, Bl), 4 Hips, 4 Knees
ISAAC_LIMITS_LOWER = np.array([
    -0.50, -0.50, -0.50, -0.50,  # Rolls (Fr, Fl, Br, Bl)
    -2.50, -2.50, -2.50, -2.50,  # Hip Pitches (Fr, Fl, Br, Bl)
    -0.25, -0.25, -0.25, -0.25,  # Knees (Fr, Fl, Br, Bl)
], dtype=np.float32)

ISAAC_LIMITS_UPPER = np.array([
    +0.50, +0.50, +0.50, +0.50,  # Rolls (Fr, Fl, Br, Bl)
    +0.20, +0.20, +0.20, +0.20,  # Hip Pitches (Fr, Fl, Br, Bl)
    +2.50, +2.50, +2.50, +2.50,  # Knees (Fr, Fl, Br, Bl)
], dtype=np.float32)

# Feedback may overshoot nominal command limits by up to 0.2 rad.
POSITION_LIMIT_TOLERANCE = 0.2
FEEDBACK_LIMITS_LOWER = ISAAC_LIMITS_LOWER - POSITION_LIMIT_TOLERANCE
FEEDBACK_LIMITS_UPPER = ISAAC_LIMITS_UPPER + POSITION_LIMIT_TOLERANCE

ACTION_SCALE = 0.25      # Policy action scaling factor
CONTROL_DT = 0.02        # 50 Hz control loop (20 ms)
COMMAND_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# Gain Scheduling Constants dynamically sourced from parameters.py (config/sim2real.yaml)

RL_KP_ROLL = float(P.KP_GAIN[0])
RL_KD_ROLL = float(P.KD_GAIN[0])
RL_KP_PITCH = float(P.KP_GAIN[1])
RL_KD_PITCH = float(P.KD_GAIN[1])

TRANSITION_KP_ROLL = float(P.KP_GAIN[0])
TRANSITION_KD_ROLL = float(P.KD_GAIN[0])
TRANSITION_KP_PITCH = float(P.KP_GAIN[1])
TRANSITION_KD_PITCH = float(P.KD_GAIN[1])

# Isaac order: [0..3 Rolls, 4..7 Hips, 8..11 Knees]
DEFAULT_TRANSITION_KP = [TRANSITION_KP_ROLL] * 4 + [TRANSITION_KP_PITCH] * 8
DEFAULT_TRANSITION_KD = [TRANSITION_KD_ROLL] * 4 + [TRANSITION_KD_PITCH] * 8

DEFAULT_RL_KP = [RL_KP_ROLL] * 4 + [RL_KP_PITCH] * 8
DEFAULT_RL_KD = [RL_KD_ROLL] * 4 + [RL_KD_PITCH] * 8

MAX_HOMING_VEL = 0.45    # rad/s (ultra-smooth continuous rate limit for startup homing)

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
        self.declare_parameter("safe_park_duration", 2.0)
        self.declare_parameter("safe_park_kp", 14.0)
        self.declare_parameter("safe_park_kd", 0.5)
        self.declare_parameter("safe_park_double_press_window", 1.0)
        self.declare_parameter("cmd_timeout", 0.25)       # Seconds of no input before zeroing cmd_vel (Watchdog)
        self.declare_parameter("action_ema_alpha", 0.0)   # Action EMA filter (0.0 = disabled, raw policy actions)

        # Gain Parameters (Stiffness & Damping)
        self.declare_parameter("rl_kp_pitch", RL_KP_PITCH)
        self.declare_parameter("rl_kd_pitch", RL_KD_PITCH)
        self.declare_parameter("rl_kp_roll", RL_KP_ROLL)
        self.declare_parameter("rl_kd_roll", RL_KD_ROLL)
        self.declare_parameter("transition_kp_pitch", TRANSITION_KP_PITCH)
        self.declare_parameter("transition_kd_pitch", TRANSITION_KD_PITCH)
        self.declare_parameter("transition_kp_roll", TRANSITION_KP_ROLL)
        self.declare_parameter("transition_kd_roll", TRANSITION_KD_ROLL)

        policy_param = self.get_parameter("policy_path").get_parameter_value().string_value
        if not policy_param:
            candidates = [
                os.path.abspath(os.path.join(os.path.dirname(__file__), "../models/policy.pt")),
            ]
            try:
                from ament_index_python.packages import get_package_share_directory
                candidates.append(os.path.join(
                    get_package_share_directory("jaguar_control"), "models", "policy.pt"))
            except Exception:
                pass
            policy_param = next((path for path in candidates if os.path.isfile(path)), candidates[0])
        if not os.path.isfile(policy_param):
            raise FileNotFoundError(f"TorchScript policy not found: {policy_param}. Set policy_path explicitly.")

        # Deployment target is the robot mini PC. Keep inference CPU-only and
        # avoid allowing Torch to consume every control-core thread.
        torch.set_num_threads(max(1, int(os.environ.get("JAGUAR_TORCH_THREADS", "1"))))

        self.torque_limit = float(self.get_parameter("torque_limit").value)
        self.shutdown_duration = float(self.get_parameter("shutdown_duration").value)
        self.shutdown_settle_delay = float(self.get_parameter("shutdown_settle_delay").value)
        self.safe_park_duration = float(self.get_parameter("safe_park_duration").value)
        self.safe_park_kp = float(self.get_parameter("safe_park_kp").value)
        self.safe_park_kd = float(self.get_parameter("safe_park_kd").value)
        self.safe_park_double_press_window = float(self.get_parameter("safe_park_double_press_window").value)
        if (self.safe_park_duration <= 0 or self.safe_park_kp < 0 or self.safe_park_kd < 0 or
                self.safe_park_double_press_window <= 0):
            raise ValueError("Invalid safe-park safety parameters")
        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)
        self.action_ema_alpha = float(self.get_parameter("action_ema_alpha").value)
        self.sitdown_settle_delay = 0.5
        self.torque_overload_cycles = 5  # 100 ms debounce at 50 Hz
        self.last_cmd_time = 0.0

        # Load Gains
        self.rl_kp_pitch = float(self.get_parameter("rl_kp_pitch").value)
        self.rl_kd_pitch = float(self.get_parameter("rl_kd_pitch").value)
        self.rl_kp_roll = float(self.get_parameter("rl_kp_roll").value)
        self.rl_kd_roll = float(self.get_parameter("rl_kd_roll").value)
        self.transition_kp_pitch = float(self.get_parameter("transition_kp_pitch").value)
        self.transition_kd_pitch = float(self.get_parameter("transition_kd_pitch").value)
        self.transition_kp_roll = float(self.get_parameter("transition_kp_roll").value)
        self.transition_kd_roll = float(self.get_parameter("transition_kd_roll").value)

        self.rl_kp = [self.rl_kp_roll] * 4 + [self.rl_kp_pitch] * 8
        self.rl_kd = [self.rl_kd_roll] * 4 + [self.rl_kd_pitch] * 8
        self.transition_kp = [self.transition_kp_roll] * 4 + [self.transition_kp_pitch] * 8
        self.transition_kd = [self.transition_kd_roll] * 4 + [self.transition_kd_pitch] * 8

        self.filtered_action = np.zeros(12, dtype=np.float32)

        self.get_logger().info(f"Loading TorchScript Policy from: {policy_param}")
        self.policy = torch.jit.load(policy_param, map_location="cpu")
        self.policy.eval()
        with torch.no_grad():
            probe = self.policy(torch.zeros((1, 5, 45), dtype=torch.float32))
        if tuple(probe.shape) != (1, 12) or not torch.isfinite(probe).all():
            raise RuntimeError(f"Policy contract failure: expected finite (1,12), got {tuple(probe.shape)}")

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
        self.last_imu_time = 0.0
        self.last_joints_time = 0.0
        self.previous_buttons = []
        self.last_safe_park_request = 0.0
        self.park_finish_time = 0.0
        self.last_sensor_wait_report = 0.0
        self.overtorque_counter = 0
        self.soft_fault_times = {}

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
        self.create_subscription(Twist, "/cmd_vel", self._cmd_vel_cb, COMMAND_QOS)
        self.create_subscription(Joy, "/joy", self._joy_cb, 10)
        self.create_subscription(Bool, "/jaguar/safe_stop", self._safe_stop_cb, 10)
        self.create_subscription(Bool, "/jaguar/emergency_stop", self._estop_cb, 10)

        self.create_subscription(Bool, "/jaguar/hardware_safe_park", self._hardware_safe_park_cb, 10)

        # Publishers
        self.safe_park_active_pub = self.create_publisher(Bool, "/jaguar/safe_park_active", 10)
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

    def _trigger_hard_estop(self, reason: str):
        if self.state == "DISABLED":
            return
        self.get_logger().error(f"[FAULT] {reason}. Motion inhibited until driver/controller restart.")
        self.state = "DISABLED"
        with self.state_lock:
            self.cmd_vel[:] = 0.0
        stop = Bool()
        stop.data = True
        self.estop_pub.publish(stop)
        passive = JointState()
        passive.name = ISAAC_JOINT_NAMES
        passive.position = [0.0] * 12
        passive.velocity = [0.0] * 12
        passive.effort = [0.0] * 24
        self.joint_cmd_pub.publish(passive)

    def _trigger_safe_shutdown(self, now: float, reason: str):
        if self.state == "SAFE_PARK" or not self._sensors_ready(check_limits=False):
            self._trigger_hard_estop(reason)
            return
        self._request_safe_park(now, reason, automatic=True)

    def _persistent_fault(self, key, bad):
        if not bad:
            self.soft_fault_times.pop(key, None)
            return False
        now = time.monotonic()
        since = self.soft_fault_times.setdefault(key, now)
        return now - since >= 0.1

    def _hardware_safe_park_cb(self, msg: Bool):
        if msg.data and self.state not in ["SAFE_PARK", "DISABLED"]:
            self._trigger_safe_shutdown(
                self.get_clock().now().nanoseconds / 1e9, "Driver motion watchdog requested SAFE_PARK")

    def _request_safe_park(self, now: float, reason: str, automatic=False):
        if self.state == "DISABLED":
            return
        if (not automatic and self.last_safe_park_request > 0.0 and
                now - self.last_safe_park_request <= self.safe_park_double_press_window):
            self._trigger_hard_estop("SAFE_PARK pressed twice: hard emergency stop")
            return
        if not automatic:
            self.last_safe_park_request = now
        if not self._sensors_ready(check_limits=False):
            self._trigger_hard_estop("SAFE_PARK requested without fresh valid sensors")
            return
        with self.state_lock:
            self.state = "SAFE_PARK"
            self.soft_fault_times.clear()
            self.overtorque_counter = 0
            self.park_initial_extra = np.maximum(
                np.maximum(ISAAC_LIMITS_LOWER - self.joint_pos,
                           self.joint_pos - ISAAC_LIMITS_UPPER), 0.0)
            self.transition_start_pos = self.joint_pos.copy()
            self.transition_target_pos = SIT_JOINT_POS.copy()
            self.transition_start_time = now
            self.park_finish_time = 0.0
            self.cmd_vel[:] = 0.0
            self.filtered_action[:] = 0.0
        self.get_logger().warn(
            f"[SAFE_PARK] {reason}. First request: moving slowly to 0 rad; "
            f"press again within {self.safe_park_double_press_window:.1f}s for HARD_ESTOP."
        )

    def _safe_stop_cb(self, msg: Bool):
        if msg.data:
            now = self.get_clock().now().nanoseconds / 1e9
            self._request_safe_park(now, "Safe park requested via topic")

    def _estop_cb(self, msg: Bool):
        if msg.data:
            self._trigger_hard_estop("Emergency stop signal received")

    def _imu_cb(self, msg: Imu):
        q = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        angular = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        if (not np.all(np.isfinite(q)) or abs(float(q @ q) - 1.0) > 0.1
                or not np.all(np.isfinite(angular)) or msg.orientation_covariance[0] == -1):
            self.imu_received = False
            return
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
            self.last_imu_time = time.monotonic()

    def _cmd_vel_cb(self, msg: Twist):
        now = self.get_clock().now().nanoseconds / 1e9
        with self.state_lock:
            cmd_deadzone = 0.10
            vx = float(np.clip(msg.linear.x, -1.0, 1.0))
            vy = float(np.clip(msg.linear.y, -1.0, 1.0))
            wz = float(np.clip(msg.angular.z, -1.0, 1.0))
            self.cmd_vel[0] = vx if abs(vx) >= cmd_deadzone else 0.0
            self.cmd_vel[1] = vy if abs(vy) >= cmd_deadzone else 0.0
            self.cmd_vel[2] = wz if abs(wz) >= cmd_deadzone else 0.0
            self.last_cmd_time = now

    def _joint_state_cb(self, msg: JointState):
        if (len(msg.position) != 12 or len(msg.velocity) != 12 or len(msg.effort) != 12
                or not np.all(np.isfinite([msg.position, msg.velocity, msg.effort]))):
            self.joints_received = False
            return
        if msg.name:
            indices = [ROS_NAME_TO_ISAAC_IDX.get(name) for name in msg.name]
            if len(indices) != 12 or None in indices or len(set(indices)) != 12:
                self.joints_received = False
                return
        else:
            indices = np.argsort(ROS_TO_ISAAC).tolist()
        with self.state_lock:
            for source, target in enumerate(indices):
                self.joint_pos[target] = msg.position[source]
                self.joint_vel[target] = msg.velocity[source]
                self.joint_tau[target] = msg.effort[source]
            self.joints_received = True
            self.last_joints_time = time.monotonic()

    def _sensors_ready(self, check_limits=True):
        now = time.monotonic()
        return (self.imu_received and self.joints_received
                and now - self.last_imu_time <= 0.25 and now - self.last_joints_time <= 0.25
                and np.all(np.isfinite(self.joint_pos))
                and np.all(self.joint_pos >= FEEDBACK_LIMITS_LOWER - (0.0 if check_limits else 0.2))
                and np.all(self.joint_pos <= FEEDBACK_LIMITS_UPPER + (0.0 if check_limits else 0.2)))

    def _publish_sensor_wait_status(self):
        now = time.monotonic()
        if now - self.last_sensor_wait_report < 1.0:
            return
        self.last_sensor_wait_report = now
        imu_age = now - self.last_imu_time if self.last_imu_time else float("inf")
        joint_age = now - self.last_joints_time if self.last_joints_time else float("inf")
        finite = bool(np.all(np.isfinite(self.joint_pos)))
        in_limits = bool(finite and np.all(self.joint_pos >= FEEDBACK_LIMITS_LOWER)
                         and np.all(self.joint_pos <= FEEDBACK_LIMITS_UPPER))
        bad_joints = []
        if finite:
            for index, value in enumerate(self.joint_pos):
                if value < FEEDBACK_LIMITS_LOWER[index] or value > FEEDBACK_LIMITS_UPPER[index]:
                    bad_joints.append(
                        f"{ISAAC_JOINT_NAMES[index]}={value:+.3f}"
                        f"[{FEEDBACK_LIMITS_LOWER[index]:+.2f},{FEEDBACK_LIMITS_UPPER[index]:+.2f}]")
        status_msg = String()
        status_msg.data = (
            f"WAITING_SENSORS state={self.state} imu={self.imu_received} "
            f"imu_age={imu_age:.2f}s joints={self.joints_received} "
            f"joint_age={joint_age:.2f}s finite={finite} in_limits={in_limits} "
            f"bad_joints={','.join(bad_joints) if bad_joints else 'none'}")
        self.status_pub.publish(status_msg)
        self.get_logger().warn(status_msg.data)

    def _joy_cb(self, msg: Joy):
        now = self.get_clock().now().nanoseconds / 1e9
        previous = self.previous_buttons
        self.previous_buttons = list(msg.buttons)
        pressed = lambda i: (i < len(msg.buttons) and msg.buttons[i] == 1
                             and (i >= len(previous) or previous[i] != 1))
        if pressed(4) or pressed(6):
            self._trigger_hard_estop("Xbox LB/Back hard emergency stop")
            return
        if pressed(5):
            self._request_safe_park(now, "Xbox RB safe-park request")
            return
        if self.state == "DISABLED" or not self._sensors_ready():
            return
        if len(msg.buttons) > 1:
            # Button 0 (X / Cross / Key '2'): Stand Up (Berdiri)
            if pressed(0) and self.state in ["STARTUP_SIT", "STANDBY", "SIT_HOLD"]:
                self.state = "STANDUP"
                self.transition_start_time = now
                with self.state_lock:
                    self.transition_start_pos = self.joint_pos.copy()
                    self.transition_target_pos = np.clip(DEFAULT_JOINT_POS.copy(), ISAAC_LIMITS_LOWER, ISAAC_LIMITS_UPPER)
                    self.cmd_vel[:] = 0.0
                    self.filtered_action[:] = 0.0
                self.get_logger().info(f"[CONTROLLER] State transition -> STANDUP ({self.transition_duration:.1f}s smooth S-curve)")
            # Button 1 (Circle / B / Key '3'): Start RL Walking (Jalan RL)
            elif pressed(1) and self.state == "STAND_HOLD":
                self.state = "WALK"
                with self.state_lock:
                    self.filtered_action[:] = 0.0
                    init_obs = self.obs_builder.build_step_observation(self.body_ang_vel, self.body_quat, self.cmd_vel, self.joint_pos, self.joint_vel)
                    self.obs_builder.reset_history(init_obs)
                ema_info = f"alpha={self.action_ema_alpha}" if self.action_ema_alpha > 0.0 else "DISABLED"
                self.get_logger().info(
                    f"[CONTROLLER] State transition -> WALK (DreamWaQ CENet | Gains: Coxa[Kp={self.rl_kp_roll}, Kd={self.rl_kd_roll}], Leg[Kp={self.rl_kp_pitch}, Kd={self.rl_kd_pitch}] | Action EMA: {ema_info})"
                )
            # Button 2 (Square / X / Key '1'): Smooth Sit Down (Duduk perlahan)
            elif pressed(2):
                if self.state in ["STANDBY", "STAND_HOLD", "STANDUP", "WALK"]:
                    self.state = "SITDOWN"
                    self.transition_start_time = now
                    with self.state_lock:
                        self.transition_start_pos = self.joint_pos.copy()
                        self.transition_target_pos = np.clip(SIT_JOINT_POS.copy(), ISAAC_LIMITS_LOWER, ISAAC_LIMITS_UPPER)
                        self.cmd_vel[:] = 0.0
                        self.filtered_action[:] = 0.0
                    self.get_logger().info(f"[CONTROLLER] State transition -> SITDOWN ({self.transition_duration:.1f}s smooth S-curve)")

        # Velocity is accepted only through /cmd_vel. Keeping mode pulses and
        # analog commands on separate topics prevents callback-order overwrites
        # and avoids applying a second joystick deadzone here.

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

        if not self._sensors_ready(check_limits=False):
            if self.state not in ["STANDBY", "DISABLED"]:
                self._trigger_hard_estop("Missing/invalid/stale sensor data or severe position excursion")
            else:
                self._publish_sensor_wait_status()
            return

        now = self.get_clock().now().nanoseconds / 1e9

        if self.state not in ["STANDBY", "DISABLED"]:
            extra = np.maximum(np.maximum(ISAAC_LIMITS_LOWER - self.joint_pos,
                                          self.joint_pos - ISAAC_LIMITS_UPPER), 0.0)
            limit = (np.maximum(POSITION_LIMIT_TOLERANCE, self.park_initial_extra + 0.05)
                     if self.state == "SAFE_PARK" else POSITION_LIMIT_TOLERANCE)
            if self._persistent_fault("position", bool(np.any(extra > limit))):
                self._trigger_safe_shutdown(now, "Persistent joint feedback position limit violation")

        # Command Watchdog: If no /joy or /cmd_vel received within cmd_timeout (0.25s), auto-zero cmd_vel
        if self.last_cmd_time > 0.0 and (now - self.last_cmd_time) > self.cmd_timeout:
            with self.state_lock:
                self.cmd_vel[:] = 0.0

        with self.state_lock:
            pos = self.joint_pos.copy()
            vel = self.joint_vel.copy()
            tau = self.joint_tau.copy()
            quat = self.body_quat.copy()
            ang_v = self.body_ang_vel.copy()
            lin_v = self.body_lin_vel.copy()
            cmd = self.cmd_vel.copy()

        # Failsafe 1: Over-Torque Protection (Continuous overload > threshold for >100ms in active states)
        max_tau = float(np.max(np.abs(tau)))
        if max_tau > self.torque_limit and self.state not in ["STANDBY", "DISABLED"]:
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
        if gz_body > -0.5 and self.state not in ["STANDBY", "DISABLED"]:
            self._trigger_safe_shutdown(now, f"Critical tilt detected (gz={gz_body:.2f} > -0.5, tilt > 60 deg)")

        target_pos = None
        target_vel = np.zeros(12, dtype=np.float32)
        cmd_kp = self.transition_kp[:]
        cmd_kd = self.transition_kd[:]

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
            cmd_kp = self.transition_kp[:]
            cmd_kd = self.transition_kd[:]
            if alpha >= 1.0:
                self.state = "STAND_HOLD"
                target_pos = DEFAULT_JOINT_POS.copy()
                target_vel = np.zeros(12, dtype=np.float32)
                self.get_logger().info("[CONTROLLER] STANDUP complete -> Holding standing pose firmly (Power ON)")
        elif self.state == "STAND_HOLD":
            # Actively hold standing pose with power ON
            target_pos = DEFAULT_JOINT_POS.copy()
            target_vel = np.zeros(12, dtype=np.float32)
            cmd_kp = self.transition_kp[:]
            cmd_kd = self.transition_kd[:]
        elif self.state == "SITDOWN":
            elapsed = now - self.transition_start_time
            alpha = float(np.clip(elapsed / self.transition_duration, 0.0, 1.0))
            smooth_alpha = 0.5 * (1.0 - math.cos(math.pi * alpha))
            diff = self.transition_target_pos - self.transition_start_pos
            target_pos = self.transition_start_pos + smooth_alpha * diff
            target_vel = (math.pi / (2.0 * self.transition_duration)) * math.sin(math.pi * alpha) * diff
            cmd_kp = self.transition_kp[:]
            cmd_kd = self.transition_kd[:]
            if alpha >= 1.0:
                target_pos = SIT_JOINT_POS.copy()
                target_vel = np.zeros(12, dtype=np.float32)
                if elapsed >= (self.transition_duration + self.sitdown_settle_delay):
                    self.state = "STANDBY"
                    cmd_kp = [0.0] * 12
                    cmd_kd = [0.0] * 12
                    self.get_logger().info("[CONTROLLER] SITDOWN complete -> Motors relaxed to STANDBY (Passive Zero-Torque)")
        elif self.state == "SAFE_PARK":
            elapsed = now - self.transition_start_time
            alpha = float(np.clip(elapsed / self.safe_park_duration, 0.0, 1.0))
            smooth_alpha = 0.5 * (1.0 - math.cos(math.pi * alpha))
            diff = self.transition_target_pos - self.transition_start_pos
            target_pos = self.transition_start_pos + smooth_alpha * diff
            target_vel = (math.pi / (2.0 * self.safe_park_duration)) * math.sin(math.pi * alpha) * diff
            cmd_kp = [self.safe_park_kp] * 12
            cmd_kd = [self.safe_park_kd] * 12
            if alpha >= 1.0:
                target_pos = SIT_JOINT_POS.copy()
                target_vel = np.zeros(12, dtype=np.float32)
                if self.park_finish_time == 0.0:
                    self.park_finish_time = now
                # Hold zero briefly, then return to passive sensing.
                if now - self.park_finish_time >= self.shutdown_settle_delay:
                    if np.max(np.abs(pos - SIT_JOINT_POS)) > 0.15:
                        self._trigger_hard_estop("SAFE_PARK failed to reach sitting pose within 0.15 rad")
                        return
                    self.state = "STANDBY"
                    cmd_kp = [0.0] * 12
                    cmd_kd = [0.0] * 12
                    self.get_logger().info("[SAFE_PARK] Position 0 rad reached; motors returned to passive mode.")

        # RL
        elif self.state == "WALK":
            # 1. Build 45-Dimensional Step Observation & Update 5-Step History Buffer (1, 5, 45)
            obs_45d = self.obs_builder.build_step_observation(ang_v, quat, cmd, pos, vel)
            history_tensor = self.obs_builder.update_and_get_history(obs_45d)

            # 2. Neural Network Forward Inference (Fused CENet + Actor JIT Model)
            with torch.no_grad():
                actions = self.policy(history_tensor)

            raw_action = actions.squeeze(0).cpu().numpy()
            if raw_action.shape != (12,) or not np.all(np.isfinite(raw_action)):
                self._trigger_safe_shutdown(now, "Invalid policy output")
                return
            self.obs_builder.update_last_action(raw_action)

            # Apply Single Action EMA Low-Pass Filter on Policy Actions (bypass if alpha <= 0.0)
            if self.action_ema_alpha > 0.0:
                self.filtered_action = (
                    self.action_ema_alpha * self.filtered_action + (1.0 - self.action_ema_alpha) * raw_action
                )
            else:
                self.filtered_action = raw_action.copy()

            target_pos = DEFAULT_JOINT_POS + ACTION_SCALE * self.filtered_action
            target_vel = np.zeros(12, dtype=np.float32)

            # Impedance tracking gains for RL (Coxa: Kp=20, Kd=1.5 | Leg: Kp=25, Kd=1.5)
            cmd_kp = self.rl_kp[:]
            cmd_kd = self.rl_kd[:]

            # 3. Publish Debug State Vector
            debug_msg = Float32MultiArray()
            debug_msg.data = obs_45d.tolist()
            self.debug_pub.publish(debug_msg)

        # Publish Joint Commands to CAN Motor Node only during active states
        if target_pos is not None:
            # Physical safety clamping to prevent exceeding mechanical limits
            target_pos = np.clip(target_pos, ISAAC_LIMITS_LOWER, ISAAC_LIMITS_UPPER)

            # Tracking error watchdog to prevent motor runaways / cable damage
            if self.state in ["STANDUP", "STAND_HOLD", "WALK", "SITDOWN", "SAFE_PARK"]:
                tracking_err = np.abs(self.joint_pos - target_pos)
                max_err = float(np.max(tracking_err))
                if self._persistent_fault("tracking", max_err > 1.57):
                    fault_idx = int(np.argmax(tracking_err))
                    self._trigger_safe_shutdown(
                        now,
                        f"Critical tracking error on {ISAAC_JOINT_NAMES[fault_idx]}: "
                        f"actual={self.joint_pos[fault_idx]:+.3f} rad "
                        f"target={target_pos[fault_idx]:+.3f} rad "
                        f"error={max_err:.2f} rad > 1.57 rad! Failsafe tripped.")
                    return

            cmd_msg = JointState()
            cmd_msg.header.stamp = self.get_clock().now().to_msg()
            cmd_msg.name = ISAAC_JOINT_NAMES
            cmd_msg.position = target_pos.tolist()
            cmd_msg.velocity = target_vel.tolist()
            cmd_msg.effort = (cmd_kp + cmd_kd)
            self.joint_cmd_pub.publish(cmd_msg)
            park_status = Bool()
            park_status.data = self.state == "SAFE_PARK"
            self.safe_park_active_pub.publish(park_status)

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
