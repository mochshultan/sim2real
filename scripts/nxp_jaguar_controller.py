#!/usr/bin/env python3
"""
NXP Jaguar Sim-to-Real Controller Node for ROS 2.
Deploys the Jaguar AdaBoot TorchScript policy to RobStride RS00 hardware.
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
from std_msgs.msg import Bool, Float32MultiArray, String
from std_srvs.srv import Trigger

from joint_layout import ISAAC_JOINT_NAMES, ROS_NAME_TO_ISAAC_IDX
import parameters as P
from motion_limits import declare_motion_limits
from policy_observation import JaguarObservationBuilder
from project_paths import package_file

# ==============================================================================
# 1. NXP JAGUAR JOINT ORDER & NOMINAL CONFIGURATION
# ==============================================================================
N_JOINTS = 12

# Remapping Indices: ROS CAN Order (BL, BR, FL, FR) <-> Isaac Lab Order
ROS_TO_ISAAC = P.ROS_TO_ISAAC

# Standby Sitting Joint Angles (0.0 rad)
SIT_JOINT_POS = np.zeros(12, dtype=np.float32)

# Standing target [rad], mapped from ROS order into Isaac order.
STAND_JOINT_POS = np.array(P.STAND_ANGLE, dtype=np.float32)[ROS_TO_ISAAC]

# Nominal reference from the deployed Jaguar AdaBoot Ideal L2C2 checkpoint 29996.
DEFAULT_JOINT_POS = np.array([
    0.0,   0.0,   0.0,   0.0,    # Rolls (Fr, Fl, Br, Bl)
   -1.51, -1.51, -1.49, -1.49,   # Hip Pitches (Fr, Fl, Br, Bl)
    1.22,  1.22,  1.20,  1.20,   # Knees (Fr, Fl, Br, Bl)
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
RL_KP_KNEE = float(P.KP_GAIN[2])
RL_KD_KNEE = float(P.KD_GAIN[2])

TRANSITION_KP_ROLL = float(P.KP_GAIN[0])
TRANSITION_KD_ROLL = float(P.KD_GAIN[0])
TRANSITION_KP_PITCH = float(P.KP_GAIN[1])
TRANSITION_KD_PITCH = float(P.KD_GAIN[1])
TRANSITION_KP_KNEE = float(P.KP_GAIN[2])
TRANSITION_KD_KNEE = float(P.KD_GAIN[2])

# Isaac order: [0..3 Rolls, 4..7 Hips, 8..11 Knees]


class NXPJaguarControllerNode(Node):
    def __init__(self):
        super().__init__("nxp_jaguar_controller")
        policy_path = self._configure_parameters()
        self._load_policy(policy_path)

        self.obs_builder = JaguarObservationBuilder(DEFAULT_JOINT_POS)
        self.state_lock = threading.Lock()

        # State Variables
        self.joint_pos = SIT_JOINT_POS.copy()
        self.joint_vel = np.zeros(12, dtype=np.float32)
        self.joint_tau = np.zeros(12, dtype=np.float32)
        self.body_quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
        self.body_ang_vel = np.zeros(3, dtype=np.float32)
        self.body_lin_vel = np.zeros(3, dtype=np.float32)
        self.cmd_vel = np.zeros(3, dtype=np.float32)
        self.last_benchmark_cmd_time = float("-inf")

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
        self.hardware_status = ""
        self.last_hardware_status_time = 0.0
        self.fault_stop_seen = False

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
        self.create_subscription(Twist, "/jaguar/benchmark_cmd_vel", self._benchmark_cmd_vel_cb, COMMAND_QOS)
        self.create_subscription(Joy, "/joy", self._joy_cb, 10)
        self.create_subscription(Bool, "/jaguar/safe_stop", self._safe_stop_cb, 10)
        self.create_subscription(Bool, "/jaguar/emergency_stop", self._estop_cb, 10)

        self.create_subscription(Bool, "/jaguar/hardware_safe_park", self._hardware_safe_park_cb, 10)
        self.create_subscription(String, "/jaguar/hardware_status", self._hardware_status_cb, 10)
        self.create_service(Trigger, "/jaguar/reset_controller", self._reset_controller_cb)

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
        self.get_logger().info(
            f"IMU standing auto-level: {'ON' if self.use_imu_stabilization else 'OFF'} | "
            f"hip tolerance={self.imu_hip_tolerance:.3f} rad | "
            f"roll tolerance={self.imu_roll_tolerance:.3f} rad | knees=fixed"
        )

    def _configure_parameters(self):
        """Declare ROS overrides and validate controller/safety configuration."""
        self.motion_limits = declare_motion_limits(self)

        # Declare parameters
        self.declare_parameter("policy_path", "")
        self.declare_parameter("torque_safety_enabled", True)
        self.declare_parameter("overtorque_threshold", 14.0)
        self.declare_parameter("shutdown_duration", 3.0)
        self.declare_parameter("shutdown_settle_delay", 0.5)
        self.declare_parameter("safe_park_duration", 2.0)
        self.declare_parameter("safe_park_kp", 14.0)
        self.declare_parameter("safe_park_kd", 0.5)
        self.declare_parameter("safe_park_double_press_window", 1.0)
        self.declare_parameter("cmd_timeout", 0.25)       # Seconds of no input before zeroing cmd_vel (Watchdog)
        self.declare_parameter("action_ema_alpha", 0.0)   # Action EMA filter (0.0 = disabled per user request)
        self.declare_parameter("walk_ramp_duration", 0.8) # Soft-start cosine ramp duration when entering WALK (s)
        self.declare_parameter("imu_upside_down", True)   # True if IMU is mounted rotated 180 deg around X on chassis

        # Gain Parameters (Stiffness & Damping)
        self.declare_parameter("rl_kp_pitch", RL_KP_PITCH)
        self.declare_parameter("rl_kd_pitch", RL_KD_PITCH)
        self.declare_parameter("rl_kp_roll", RL_KP_ROLL)
        self.declare_parameter("rl_kd_roll", RL_KD_ROLL)
        self.declare_parameter("rl_kp_knee", RL_KP_KNEE)
        self.declare_parameter("rl_kd_knee", RL_KD_KNEE)
        self.declare_parameter("transition_kp_pitch", TRANSITION_KP_PITCH)
        self.declare_parameter("transition_kd_pitch", TRANSITION_KD_PITCH)
        self.declare_parameter("transition_kp_roll", TRANSITION_KP_ROLL)
        self.declare_parameter("transition_kd_roll", TRANSITION_KD_ROLL)
        self.declare_parameter("transition_kp_knee", TRANSITION_KP_KNEE)
        self.declare_parameter("transition_kd_knee", TRANSITION_KD_KNEE)
        self.declare_parameter("use_imu_stabilization", True)
        self.declare_parameter("imu_kp_pitch", 0.55)
        self.declare_parameter("imu_kd_pitch", 0.04)
        self.declare_parameter("imu_kp_roll", 0.30)
        self.declare_parameter("imu_kd_roll", 0.03)
        self.declare_parameter("imu_hip_tolerance", 0.15)
        self.declare_parameter("imu_roll_tolerance", 0.12)
        self.declare_parameter("imu_tilt_deadband", 0.02)

        policy_param = self.get_parameter("policy_path").get_parameter_value().string_value
        if not policy_param:
            policy_param = str(package_file("models", "policy.pt"))
        if not os.path.isfile(policy_param):
            raise FileNotFoundError(f"TorchScript policy not found: {policy_param}. Set policy_path explicitly.")

        # Deployment target is the robot mini PC. Keep inference CPU-only and
        # avoid allowing Torch to consume every control-core thread.
        torch.set_num_threads(max(1, int(os.environ.get("JAGUAR_TORCH_THREADS", "1"))))

        self.torque_safety_enabled = bool(self.get_parameter("torque_safety_enabled").value)
        self.overtorque_threshold = float(self.get_parameter("overtorque_threshold").value)
        self.shutdown_duration = float(self.get_parameter("shutdown_duration").value)
        self.shutdown_settle_delay = float(self.get_parameter("shutdown_settle_delay").value)
        self.safe_park_duration = float(self.get_parameter("safe_park_duration").value)
        self.safe_park_kp = float(self.get_parameter("safe_park_kp").value)
        self.safe_park_kd = float(self.get_parameter("safe_park_kd").value)
        self.safe_park_double_press_window = float(self.get_parameter("safe_park_double_press_window").value)
        if (not 0.0 < self.overtorque_threshold <= P.RS00_TORQUE_LIMIT_NM or
                self.safe_park_duration <= 0 or self.safe_park_kp < 0 or self.safe_park_kd < 0 or
                self.safe_park_double_press_window <= 0):
            raise ValueError("Invalid torque or safe-park safety parameters")
        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)
        self.action_ema_alpha = float(self.get_parameter("action_ema_alpha").value)
        self.walk_ramp_duration = float(self.get_parameter("walk_ramp_duration").value)
        self.imu_upside_down = bool(self.get_parameter("imu_upside_down").value)
        self.walk_start_time = 0.0
        self.sitdown_settle_delay = 0.5
        self.torque_overload_cycles = 5  # 100 ms debounce at 50 Hz
        logger = self.get_logger()
        log_config = logger.info if self.torque_safety_enabled else logger.warn
        log_config(f"event=CONFIG code=OVERTORQUE enabled={int(self.torque_safety_enabled)} "
                   f"limit_nm={self.overtorque_threshold:.3f} required_samples={self.torque_overload_cycles}")
        self.last_cmd_time = 0.0

        # Load Gains
        self.rl_kp_pitch = float(self.get_parameter("rl_kp_pitch").value)
        self.rl_kd_pitch = float(self.get_parameter("rl_kd_pitch").value)
        self.rl_kp_roll = float(self.get_parameter("rl_kp_roll").value)
        self.rl_kd_roll = float(self.get_parameter("rl_kd_roll").value)
        self.rl_kp_knee = float(self.get_parameter("rl_kp_knee").value)
        self.rl_kd_knee = float(self.get_parameter("rl_kd_knee").value)
        self.transition_kp_pitch = float(self.get_parameter("transition_kp_pitch").value)
        self.transition_kd_pitch = float(self.get_parameter("transition_kd_pitch").value)
        self.transition_kp_roll = float(self.get_parameter("transition_kp_roll").value)
        self.transition_kd_roll = float(self.get_parameter("transition_kd_roll").value)
        self.transition_kp_knee = float(self.get_parameter("transition_kp_knee").value)
        self.transition_kd_knee = float(self.get_parameter("transition_kd_knee").value)
        self.use_imu_stabilization = bool(self.get_parameter("use_imu_stabilization").value)
        self.imu_kp_pitch = float(self.get_parameter("imu_kp_pitch").value)
        self.imu_kd_pitch = float(self.get_parameter("imu_kd_pitch").value)
        self.imu_kp_roll = float(self.get_parameter("imu_kp_roll").value)
        self.imu_kd_roll = float(self.get_parameter("imu_kd_roll").value)
        self.imu_hip_tolerance = float(self.get_parameter("imu_hip_tolerance").value)
        self.imu_roll_tolerance = float(self.get_parameter("imu_roll_tolerance").value)
        self.imu_tilt_deadband = float(self.get_parameter("imu_tilt_deadband").value)
        imu_values = (
            self.imu_kp_pitch, self.imu_kd_pitch, self.imu_kp_roll, self.imu_kd_roll,
            self.imu_hip_tolerance, self.imu_roll_tolerance, self.imu_tilt_deadband,
        )
        if not all(np.isfinite(value) and value >= 0.0 for value in imu_values):
            raise ValueError("IMU stabilization gains, tolerances, and deadband must be finite and non-negative")

        self.rl_kp = [self.rl_kp_roll] * 4 + [self.rl_kp_pitch] * 4 + [self.rl_kp_knee] * 4
        self.rl_kd = [self.rl_kd_roll] * 4 + [self.rl_kd_pitch] * 4 + [self.rl_kd_knee] * 4
        self.transition_kp = [self.transition_kp_roll] * 4 + [self.transition_kp_pitch] * 4 + [self.transition_kp_knee] * 4
        self.transition_kd = [self.transition_kd_roll] * 4 + [self.transition_kd_pitch] * 4 + [self.transition_kd_knee] * 4

        self.filtered_action = np.zeros(12, dtype=np.float32)

        return policy_param

    def _load_policy(self, policy_param):
        """Load and verify the CPU inference contract before enabling callbacks."""
        self.get_logger().info(f"Loading TorchScript Policy from: {policy_param}")
        self.policy = torch.jit.load(policy_param, map_location="cpu")
        self.policy.eval()
        self.policy_expects_temporal = True
        with torch.no_grad():
            try:
                probe = self.policy(torch.zeros((1, 5, 45), dtype=torch.float32))
                if tuple(probe.shape) == (1, 12) and torch.isfinite(probe).all():
                    self.policy_expects_temporal = True
                else:
                    self.policy_expects_temporal = False
            except Exception:
                self.policy_expects_temporal = False

            if not self.policy_expects_temporal:
                probe = self.policy(torch.zeros((1, 45), dtype=torch.float32))

        if tuple(probe.shape) != (1, 12) or not torch.isfinite(probe).all():
            raise RuntimeError(f"Policy contract failure: expected finite (1,12), got {tuple(probe.shape)}")
        self.get_logger().info(f"Policy verified: expects_temporal={self.policy_expects_temporal}, output_shape={tuple(probe.shape)}")

    def _log_safety(self, event: str, reason: str, state=None):
        fields = tuple(part for part in reason.split()
                       if part.startswith(("code=", "joint=", "sensor=", "source=")))
        key = (event, fields or (reason,))
        if not hasattr(self, "_safety_log_keys"):
            self._safety_log_keys = set()
        if key in self._safety_log_keys:
            return
        self._safety_log_keys.add(key)
        message = f"event={event} state={self.state if state is None else state} {reason}"
        logger = self.get_logger()
        (logger.error if event == "ESTOP" else logger.warn)(message)

    def _trigger_hard_estop(self, reason: str):
        self._log_safety("ESTOP", reason)
        if self.state == "DISABLED":
            return
        self.state = "DISABLED"
        self.fault_stop_seen = False
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

    def _hardware_status_cb(self, msg: String):
        self.hardware_status = msg.data
        self.last_hardware_status_time = time.monotonic()
        if self.state == "DISABLED" and msg.data in ("EMERGENCY_STOPPED", "RESETTING_FAULT"):
            self.fault_stop_seen = True

    def _reset_controller_cb(self, request, response):
        del request
        hardware_ready = (self.hardware_status == "PASSIVE_ZERO_TORQUE"
                          and time.monotonic() - self.last_hardware_status_time <= 2.0)
        if (self.state != "DISABLED" or not self.fault_stop_seen
                or not hardware_ready or not self._sensors_ready()):
            response.success = False
            response.message = "Reset rejected: controller must be DISABLED, hardware passive, and all sensors fresh/in limits."
            return response
        with self.state_lock:
            self.cmd_vel[:] = 0.0
            self.filtered_action[:] = 0.0
            self.transition_start_pos = self.joint_pos.copy()
            self.transition_target_pos = self.joint_pos.copy()
            self.obs_builder = JaguarObservationBuilder(DEFAULT_JOINT_POS)
        self.overtorque_counter = 0
        self.soft_fault_times.clear()
        self.last_safe_park_request = 0.0
        self.fault_stop_seen = False
        self.state = "STANDBY"
        self._safety_log_keys = set()
        response.success = True
        response.message = "Controller reset to STANDBY (passive). Re-arm motion manually."
        self.get_logger().info("event=RESET_COMPLETE state=STANDBY")
        return response

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

    def _update_overtorque_safety(self, now: float, tau):
        """Detect sustained overloads; shut down or only log according to config."""
        max_tau = float(np.max(np.abs(tau)))
        if max_tau > self.overtorque_threshold and self.state not in ["STANDBY", "DISABLED"]:
            self.overtorque_counter += 1
            if self.overtorque_counter >= self.torque_overload_cycles:
                joint_idx = int(np.argmax(np.abs(tau)))
                joint_name = ISAAC_JOINT_NAMES[joint_idx]
                reason = (f"code=OVERTORQUE joint={joint_name} actual_nm={max_tau:.3f} "
                          f"limit_nm={self.overtorque_threshold:.3f} samples={self.overtorque_counter} "
                          f"required_samples={self.torque_overload_cycles}")
                if self.torque_safety_enabled:
                    self._trigger_safe_shutdown(now, reason)
                else:
                    self.get_logger().warn(
                        f"event=WARNING state={self.state} {reason} action=LOG_ONLY",
                        throttle_duration_sec=1.0,
                    )
        else:
            self.overtorque_counter = max(0, self.overtorque_counter - 1)
        return max_tau

    def _hardware_safe_park_cb(self, msg: Bool):
        if msg.data and self.state not in ["SAFE_PARK", "DISABLED"]:
            self._trigger_safe_shutdown(
                self.get_clock().now().nanoseconds / 1e9, "code=DRIVER_REQUEST source=hardware_safe_park")

    def _request_safe_park(self, now: float, reason: str, automatic=False):
        if self.state == "DISABLED":
            return
        if (not automatic and self.last_safe_park_request > 0.0 and
                now - self.last_safe_park_request <= self.safe_park_double_press_window):
            self._trigger_hard_estop(
                f"code=PARK_DOUBLE_PRESS interval_s={now - self.last_safe_park_request:.3f} "
                f"limit_s={self.safe_park_double_press_window:.3f}")
            return
        if not automatic:
            self.last_safe_park_request = now
        if not self._sensors_ready(check_limits=False):
            self._trigger_hard_estop(f"code=PARK_SENSOR_INVALID {reason.replace('code=', 'request_code=', 1)}")
            for fault in self._sensor_fault_reasons():
                self._trigger_hard_estop(fault)
            return
        previous_state = self.state
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
        self._log_safety("SAFE_PARK", reason, state=previous_state)

    def _safe_stop_cb(self, msg: Bool):
        if msg.data:
            now = self.get_clock().now().nanoseconds / 1e9
            self._request_safe_park(now, "code=MANUAL source=safe_stop_topic")

    def _estop_cb(self, msg: Bool):
        if msg.data:
            self._trigger_hard_estop("code=MANUAL source=emergency_stop_topic")

    def _imu_cb(self, msg: Imu):
        q = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        angular = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        if (not np.all(np.isfinite(q)) or abs(float(q @ q) - 1.0) > 0.1
                or not np.all(np.isfinite(angular)) or msg.orientation_covariance[0] == -1):
            self.imu_received = False
            self.imu_fault_detail = (
                f"quat_norm_sq={float(q @ q):.6f} quat_finite={int(np.all(np.isfinite(q)))} "
                f"gyro_finite={int(np.all(np.isfinite(angular)))} covariance0={msg.orientation_covariance[0]} norm_tolerance=0.100000")
            return
        with self.state_lock:
            qx = msg.orientation.x
            qy = msg.orientation.y
            qz = msg.orientation.z
            qw = msg.orientation.w

            if self.imu_upside_down:
                # Reorient IMU frame if mounted upside-down (Roll 180 deg: q_body = q_raw * q_rotX(pi))
                body_qx = qw
                body_qy = qz
                body_qz = -qy
                body_qw = -qx

                self.body_ang_vel[0] = msg.angular_velocity.x
                self.body_ang_vel[1] = -msg.angular_velocity.y
                self.body_ang_vel[2] = -msg.angular_velocity.z
            else:
                body_qx = qx
                body_qy = qy
                body_qz = qz
                body_qw = qw

                self.body_ang_vel[0] = msg.angular_velocity.x
                self.body_ang_vel[1] = msg.angular_velocity.y
                self.body_ang_vel[2] = msg.angular_velocity.z

            self.body_quat[0] = body_qx
            self.body_quat[1] = body_qy
            self.body_quat[2] = body_qz
            self.body_quat[3] = body_qw

            self.imu_received = True
            self.last_imu_time = time.monotonic()

    def _cmd_vel_cb(self, msg: Twist):
        now = self.get_clock().now().nanoseconds / 1e9
        with self.state_lock:
            if time.monotonic() - self.last_benchmark_cmd_time <= self.cmd_timeout:
                return
            vx = float(np.clip(msg.linear.x, -self.motion_limits["max_vx"], self.motion_limits["max_vx"]))
            vy = float(np.clip(msg.linear.y, -self.motion_limits["max_vy"], self.motion_limits["max_vy"]))
            wz = float(np.clip(msg.angular.z, -self.motion_limits["max_wz"], self.motion_limits["max_wz"]))
            self.cmd_vel[0] = vx
            self.cmd_vel[1] = vy
            self.cmd_vel[2] = wz
            self.last_cmd_time = now

    def _benchmark_cmd_vel_cb(self, msg: Twist):
        now = self.get_clock().now().nanoseconds / 1e9
        with self.state_lock:
            self.cmd_vel[0] = float(np.clip(msg.linear.x, -self.motion_limits["max_vx"], self.motion_limits["max_vx"]))
            self.cmd_vel[1] = float(np.clip(msg.linear.y, -self.motion_limits["max_vy"], self.motion_limits["max_vy"]))
            self.cmd_vel[2] = float(np.clip(msg.angular.z, -self.motion_limits["max_wz"], self.motion_limits["max_wz"]))
            self.last_cmd_time = now
            self.last_benchmark_cmd_time = time.monotonic()

    def _joint_state_cb(self, msg: JointState):
        if len(msg.position) != 12 or len(msg.velocity) != 12 or len(msg.effort) != 12:
            self.joints_received = False
            self.joint_fault_detail = (f"reason=dimensions positions={len(msg.position)} "
                                       f"velocities={len(msg.velocity)} efforts={len(msg.effort)} expected=12")
            return
        values = np.asarray([msg.position, msg.velocity, msg.effort])
        if not np.all(np.isfinite(values)):
            self.joints_received = False
            entries = ",".join(f"{('position', 'velocity', 'effort')[field]}:{index}:{values[field, index]}"
                               for field, index in np.argwhere(~np.isfinite(values)))
            self.joint_fault_detail = f"reason=nonfinite entries={entries}"
            return
        if msg.name:
            indices = [ROS_NAME_TO_ISAAC_IDX.get(name) for name in msg.name]
            if len(indices) != 12 or None in indices or len(set(indices)) != 12:
                self.joints_received = False
                self.joint_fault_detail = f"reason=joint_names names={','.join(msg.name)}"
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

    def _compute_imu_stabilization(self, alpha: float, state: str) -> np.ndarray:
        """Return bounded standing trim in Isaac order; knee trim is always zero."""
        delta_q = np.zeros(N_JOINTS, dtype=np.float32)
        if not self.use_imu_stabilization or state not in ("STANDUP", "STAND_HOLD"):
            return delta_q

        if not self.imu_received or time.monotonic() - self.last_imu_time > 0.25:
            return delta_q

        if state == "STANDUP":
            # Introduce leveling only after the legs begin carrying the body.
            weight = float(np.clip((alpha - 0.15) / 0.35, 0.0, 1.0))
        else:
            weight = 1.0
        if weight <= 0.0:
            return delta_q

        qx, qy, qz, qw = self.body_quat
        gx = -2.0 * (qx * qz - qw * qy)
        gy = -2.0 * (qy * qz + qw * qx)
        gz = -(1.0 - 2.0 * (qx * qx + qy * qy))

        # Fade correction near the emergency tilt envelope instead of driving
        # aggressively when the robot is already close to falling.
        if gz > -0.70:
            weight *= max(0.0, float(np.clip((-gz - 0.50) / 0.20, 0.0, 1.0)))
        if weight <= 0.0:
            return delta_q

        def deadband(value: float) -> float:
            magnitude = max(0.0, abs(value) - self.imu_tilt_deadband)
            return math.copysign(magnitude, value) if magnitude else 0.0

        e_pitch = deadband(float(gx))
        e_roll = deadband(float(-gy))
        wx = float(self.body_ang_vel[0])
        wy = float(self.body_ang_vel[1])

        hip_trim = weight * (self.imu_kp_pitch * e_pitch + self.imu_kd_pitch * wy)
        roll_trim = weight * (self.imu_kp_roll * e_roll + self.imu_kd_roll * wx)
        hip_trim = float(np.clip(hip_trim, -self.imu_hip_tolerance, self.imu_hip_tolerance))
        roll_trim = float(np.clip(roll_trim, -self.imu_roll_tolerance, self.imu_roll_tolerance))

        # Isaac order: FR, FL, BR, BL rolls; then hips; then knees.
        delta_q[0:4] = [roll_trim, -roll_trim, roll_trim, -roll_trim]
        delta_q[4:8] = -hip_trim
        # Intentionally never compensate with knees. Their transition/hold
        # targets remain exactly the nominal trajectory values.
        delta_q[8:12] = 0.0
        return delta_q

    def _sensor_fault_reasons(self):
        now = time.monotonic()
        reasons = []
        for sensor, received, stamp, detail in (
            ("imu", self.imu_received, self.last_imu_time, getattr(self, "imu_fault_detail", "reason=no_valid_sample")),
            ("joints", self.joints_received, self.last_joints_time, getattr(self, "joint_fault_detail", "reason=no_valid_sample")),
        ):
            if not received:
                reasons.append(f"code=SENSOR_INVALID sensor={sensor} {detail}")
            if now - stamp > 0.25:
                reasons.append(f"code=SENSOR_TIMEOUT sensor={sensor} age_s={now - stamp:.3f} limit_s=0.250")
        for index, position in enumerate(self.joint_pos):
            if not np.isfinite(position):
                reasons.append(f"code=POSITION_NONFINITE joint={ISAAC_JOINT_NAMES[index]} actual_rad={position}")
            elif position < FEEDBACK_LIMITS_LOWER[index] - 0.2 or position > FEEDBACK_LIMITS_UPPER[index] + 0.2:
                reasons.append(
                    f"code=POSITION_RANGE joint={ISAAC_JOINT_NAMES[index]} actual_rad={position:.4f} "
                    f"min_rad={FEEDBACK_LIMITS_LOWER[index] - 0.2:.4f} max_rad={FEEDBACK_LIMITS_UPPER[index] + 0.2:.4f}")
        return reasons

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
            self._trigger_hard_estop("code=MANUAL source=xbox_lb_back")
            return
        if pressed(5):
            self._request_safe_park(now, "code=MANUAL source=xbox_rb")
            return
        if self.state == "DISABLED" or not self._sensors_ready():
            return
        if pressed(7) and self.state == "WALK":
            self.state = "STANDUP"
            self.transition_start_time = now
            with self.state_lock:
                self.transition_start_pos = self.joint_pos.copy()
                self.transition_target_pos = np.clip(STAND_JOINT_POS.copy(), ISAAC_LIMITS_LOWER, ISAAC_LIMITS_UPPER)
                self.cmd_vel[:] = 0.0
                self.filtered_action[:] = 0.0
            self.get_logger().info("event=MODE_CHANGE source=start from=WALK to=STANDUP target=STAND_HOLD")
            return
        if len(msg.buttons) > 1:
            # Button 0 (X / Cross / Key '2'): Stand Up (Berdiri)
            if pressed(0) and self.state in ["STARTUP_SIT", "STANDBY", "SIT_HOLD"]:
                self.state = "STANDUP"
                self.transition_start_time = now
                with self.state_lock:
                    self.transition_start_pos = self.joint_pos.copy()
                    self.transition_target_pos = np.clip(STAND_JOINT_POS.copy(), ISAAC_LIMITS_LOWER, ISAAC_LIMITS_UPPER)
                    self.cmd_vel[:] = 0.0
                    self.filtered_action[:] = 0.0
                self.get_logger().info(f"[CONTROLLER] State transition -> STANDUP ({self.transition_duration:.1f}s smooth S-curve)")
            # Start toggles policy; legacy button 1 / keyboard '3' enters WALK.
            elif (pressed(1) or pressed(7)) and self.state == "STAND_HOLD":
                # Pre-flight safety check: verify projected gravity before enabling WALK
                bx, by, bz, bw = self.body_quat
                gz = -(1.0 - 2.0 * (bx * bx + by * by))
                if gz > -0.5:
                    self.get_logger().error(
                        f"[CONTROLLER ERROR] Refusing to start WALK! Projected gravity gz={gz:+.2f} > -0.5 "
                        f"(robot upside-down or IMU inversion incorrect!). Expected gz ≈ -1.0."
                    )
                    return
                self.state = "WALK"
                self.walk_start_time = now
                with self.state_lock:
                    self.filtered_action[:] = 0.0
                    self.cmd_vel[:] = 0.0
                    init_obs = self.obs_builder.build_step_observation(self.body_ang_vel, self.body_quat, self.cmd_vel, self.joint_pos, self.joint_vel)
                    self.obs_builder.reset_history(init_obs)
                ema_info = f"alpha={self.action_ema_alpha:.2f}" if self.action_ema_alpha > 0.0 else "DISABLED"
                self.get_logger().info(
                    f"[CONTROLLER] State transition -> WALK (Baseline PPO | Ramp: {self.walk_ramp_duration:.1f}s | Action EMA: {ema_info} | Init gz: {gz:+.2f} | Gains: Coxa[Kp={self.rl_kp_roll}, Kd={self.rl_kd_roll}], Hip[Kp={self.rl_kp_pitch}, Kd={self.rl_kd_pitch}], Knee[Kp={self.rl_kp_knee}, Kd={self.rl_kd_knee}])"
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

        # Velocity comes from /cmd_vel or the benchmark override. Mode and safety
        # buttons remain on their separate topics regardless of command source.

    def _transition_targets(self, now, duration):
        elapsed = now - self.transition_start_time
        alpha = float(np.clip(elapsed / duration, 0.0, 1.0))
        smooth_alpha = 0.5 * (1.0 - math.cos(math.pi * alpha))
        diff = self.transition_target_pos - self.transition_start_pos
        target_pos = self.transition_start_pos + smooth_alpha * diff
        target_vel = (math.pi / (2.0 * duration)) * math.sin(math.pi * alpha) * diff
        return elapsed, alpha, target_pos, target_vel

    def _walk_targets(self, now, pos, vel, quat, ang_v, cmd):
        """Run policy inference and blend its targets with the standing pose."""
        # 1. Build 45-Dimensional Step Observation & Update 5-Step History Buffer (1, 5, 45)
        obs_45d = self.obs_builder.build_step_observation(ang_v, quat, cmd, pos, vel)
        history_tensor = self.obs_builder.update_and_get_history(obs_45d)

        # 2. Neural Network Forward Inference
        with torch.no_grad():
            if self.policy_expects_temporal:
                actions = self.policy(history_tensor)
            else:
                obs_tensor = torch.from_numpy(obs_45d).unsqueeze(0).float()
                actions = self.policy(obs_tensor)

        raw_action = actions.squeeze(0).cpu().numpy()
        if raw_action.shape != (12,) or not np.all(np.isfinite(raw_action)):
            self._trigger_safe_shutdown(now, f"code=POLICY_OUTPUT shape={'x'.join(map(str, raw_action.shape)) or 'scalar'} expected_shape=12 finite={int(np.all(np.isfinite(raw_action)))}")
            return
        self.obs_builder.update_last_action(raw_action)

        # Apply Single Action EMA Low-Pass Filter on Policy Actions (bypass if alpha <= 0.0)
        if self.action_ema_alpha > 0.0:
            self.filtered_action = (
                self.action_ema_alpha * self.filtered_action + (1.0 - self.action_ema_alpha) * raw_action
            )
        else:
            self.filtered_action = raw_action.copy()

        # Smooth soft-start / ramp into WALK mode (cosine S-curve) to eliminate initial step shock
        walk_elapsed = now - self.walk_start_time
        if self.walk_ramp_duration > 0.0 and walk_elapsed < self.walk_ramp_duration:
            ramp = 0.5 * (1.0 - math.cos(math.pi * (walk_elapsed / self.walk_ramp_duration)))
        else:
            ramp = 1.0

        applied_action = ramp * self.filtered_action
        target_pos = ((1.0 - ramp) * STAND_JOINT_POS
                      + ramp * DEFAULT_JOINT_POS + ACTION_SCALE * applied_action)
        target_vel = np.zeros(12, dtype=np.float32)

        # Impedance tracking gains for RL with smooth damping transition
        cmd_kp = self.rl_kp[:]
        cmd_kd = [
            float((1.0 - ramp) * tkd + ramp * rkd)
            for tkd, rkd in zip(self.transition_kd, self.rl_kd)
        ]

        # 3. Publish Debug State Vector
        debug_msg = Float32MultiArray()
        debug_msg.data = obs_45d.tolist()
        self.debug_pub.publish(debug_msg)
        return target_pos, target_vel, cmd_kp, cmd_kd

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
                for fault in self._sensor_fault_reasons():
                    self._trigger_hard_estop(fault)
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
                faults = []
                for index in np.flatnonzero(extra > limit):
                    bound = float(limit[index]) if np.ndim(limit) else float(limit)
                    faults.append(
                        f"code=POSITION_LIMIT joint={ISAAC_JOINT_NAMES[index]} actual_rad={self.joint_pos[index]:.4f} "
                        f"min_rad={ISAAC_LIMITS_LOWER[index]:.4f} max_rad={ISAAC_LIMITS_UPPER[index]:.4f} "
                        f"excess_rad={extra[index]:.4f} limit_rad={bound:.4f} "
                        f"duration_s={time.monotonic() - self.soft_fault_times['position']:.3f} debounce_s=0.100")
                previous_state = self.state
                self._trigger_safe_shutdown(now, faults[0])
                for fault in faults[1:]:
                    self._log_safety("VIOLATION", fault, state=previous_state)

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
            cmd = self.cmd_vel.copy()

        # Failsafe 1: sustained overtorque requests SAFE_PARK (then E-stop if
        # repeated during parking), or only logs when torque safety is disabled.
        max_tau = self._update_overtorque_safety(now, tau)

        # Failsafe 2: Tilt Safety Protection (Emergency sit if tilt > 60 deg, gz > -0.5 in active states)
        gz_body = -(1.0 - 2.0 * (quat[0]**2 + quat[1]**2))
        if gz_body > -0.5 and self.state not in ["STANDBY", "DISABLED"]:
            self._trigger_safe_shutdown(now, f"code=TILT gz={gz_body:.4f} limit_gz=-0.5000")

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
            elapsed, alpha, target_pos, target_vel = self._transition_targets(now, self.transition_duration)
            target_pos += self._compute_imu_stabilization(alpha, "STANDUP")
            cmd_kp = self.transition_kp[:]
            cmd_kd = self.transition_kd[:]
            if alpha >= 1.0:
                self.state = "STAND_HOLD"
                target_pos = STAND_JOINT_POS.copy() + self._compute_imu_stabilization(1.0, "STAND_HOLD")
                target_vel = np.zeros(12, dtype=np.float32)
                self.get_logger().info("[CONTROLLER] STANDUP complete -> Holding standing pose (Power ON + IMU Auto-Level)")
        elif self.state == "STAND_HOLD":
            # Hold nominal knees while applying bounded hip/collar leveling.
            target_pos = STAND_JOINT_POS.copy() + self._compute_imu_stabilization(1.0, "STAND_HOLD")
            target_vel = np.zeros(12, dtype=np.float32)
            cmd_kp = self.transition_kp[:]
            cmd_kd = self.transition_kd[:]
            # Publish 45D observation state during STAND_HOLD for continuous logging/benchmarking
            obs_45d = self.obs_builder.build_step_observation(ang_v, quat, cmd, pos, vel)
            debug_msg = Float32MultiArray()
            debug_msg.data = obs_45d.tolist()
            self.debug_pub.publish(debug_msg)
        elif self.state == "SITDOWN":
            elapsed, alpha, target_pos, target_vel = self._transition_targets(now, self.transition_duration)
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
            elapsed, alpha, target_pos, target_vel = self._transition_targets(now, self.safe_park_duration)
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
                        index = int(np.argmax(np.abs(pos - SIT_JOINT_POS)))
                        self._trigger_hard_estop(
                            f"code=PARK_POSITION joint={ISAAC_JOINT_NAMES[index]} actual_rad={pos[index]:.4f} "
                            f"target_rad={SIT_JOINT_POS[index]:.4f} error_rad={abs(pos[index] - SIT_JOINT_POS[index]):.4f} "
                            f"limit_rad=0.1500 elapsed_s={elapsed:.3f}")
                        return
                    self.state = "STANDBY"
                    cmd_kp = [0.0] * 12
                    cmd_kd = [0.0] * 12
                    self.get_logger().info("event=PARK_COMPLETE state=STANDBY")
                    self._safety_log_keys = set()

        # RL
        elif self.state == "WALK":
            targets = self._walk_targets(now, pos, vel, quat, ang_v, cmd)
            if targets is None:
                return
            target_pos, target_vel, cmd_kp, cmd_kd = targets

        if not self._publish_targets(target_pos, target_vel, cmd_kp, cmd_kd, now):
            return
        self._report_loop_status(t_start, cmd, max_tau)

    def _publish_targets(self, target_pos, target_vel, cmd_kp, cmd_kd, now):
        """Clamp and check tracking before publishing motor targets."""
        # Publish passive zero-gain commands in standby and targets in active states
        if target_pos is not None:
            # Physical safety clamping to prevent exceeding mechanical limits
            target_pos = np.clip(target_pos, ISAAC_LIMITS_LOWER, ISAAC_LIMITS_UPPER)

            # Tracking error watchdog to prevent motor runaways / cable damage
            if self.state in ["STANDUP", "STAND_HOLD", "WALK", "SITDOWN", "SAFE_PARK"]:
                tracking_err = np.abs(self.joint_pos - target_pos)
                max_err = float(np.max(tracking_err))
                if self._persistent_fault("tracking", max_err > 2.0):
                    faults = [
                        f"code=TRACKING joint={ISAAC_JOINT_NAMES[index]} actual_rad={self.joint_pos[index]:.4f} "
                        f"target_rad={target_pos[index]:.4f} error_rad={tracking_err[index]:.4f} limit_rad=2.0000 "
                        f"duration_s={time.monotonic() - self.soft_fault_times['tracking']:.3f} debounce_s=0.100"
                        for index in np.flatnonzero(tracking_err > 2.0)
                    ]
                    previous_state = self.state
                    self._trigger_safe_shutdown(now, faults[0])
                    for fault in faults[1:]:
                        self._log_safety("VIOLATION", fault, state=previous_state)
                    return False

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

        return True

    def _report_loop_status(self, t_start, cmd, max_tau):
        """Publish diagnostics and enforce the existing timing watchdog."""
        t_end = time.perf_counter()
        self.compute_latency_ms = (t_end - t_start) * 1000.0
        now_sec = self.get_clock().now().nanoseconds / 1e9

        # Frequency & Compute Latency Watchdog / Warnings
        if self.compute_latency_ms > 20.0 or self.actual_freq < 45.0:
            self.lag_counter += 1
            if now_sec - self.last_warn_time > 1.0:
                self.get_logger().warn(
                    f"event=TIMING latency_ms={self.compute_latency_ms:.3f} frequency_hz={self.actual_freq:.3f} "
                    f"count={self.lag_counter}"
                )
                self.last_warn_time = now_sec

            # Critical Safety Fallback: If severe lag occurs in WALK mode (compute > 45ms or freq < 25Hz for 5 consecutive loops)
            if self.state == "WALK" and (self.compute_latency_ms > 45.0 or self.actual_freq < 25.0):
                if self.lag_counter >= 5:
                    self._trigger_safe_shutdown(now_sec,
                        f"code=COMPUTE_LAG latency_ms={self.compute_latency_ms:.3f} limit_ms=45.000 "
                        f"frequency_hz={self.actual_freq:.3f} minimum_hz=25.000 count={self.lag_counter} required_count=5")
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
