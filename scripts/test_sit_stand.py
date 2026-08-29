#!/usr/bin/env python3
"""
🐾 NXP Jaguar: Standalone Sit & Stand Transition Tester (CAN Direct)
Interactive Terminal Control & Bluetooth/USB Xbox Gamepad interface for testing smooth,
non-aggressive Sit (0.0 rad) and Stand (Front Hip -1.5, Front Knee +1.4, Rear Hip -1.4, Rear Knee +1.36) transitions.

Supports:
- Direct Linux Bluetooth & USB Xbox Gamepad (/dev/input/js*)
- Non-blocking SSH Terminal keyboard control
- ROS 2 Bridge (/joint_states @ 50 Hz, /Imu_data, /joy)
- Smooth S-curve (cosine) trajectory generator with zero start/end jerk
"""

import sys
import os
import time
import math
import select
import termios
import tty
import atexit
import threading
import numpy as np
from typing import Optional, List, Dict, Any

# Add scripts directory to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import parameters as P
from robstride_motor_lib import RobStrideMotorController
from gamepad_reader import LinuxGamepadReader, XboxState

# ==============================================================================
# TARGET POSES (In ROS Joint Order: BL, BR, FL, FR)
# Note: Zero position is calibrated at folded/sitting pose.
# ==============================================================================
SIT_POSE = np.array([
    0.0,  0.0,  0.0,  # BL: collar, hip, knee
    0.0,  0.0,  0.0,  # BR: collar, hip, knee
    0.0,  0.0,  0.0,  # FL: collar, hip, knee
    0.0,  0.0,  0.0,  # FR: collar, hip, knee
], dtype=np.float64)

# Standing pose (Kaki belakang: Hip -1.50 rad, Knee +1.40 rad; Kaki depan: Hip -1.60 rad, Knee +1.70 rad)
STAND_POSE = np.array([
    0.0, -1.50,  1.40,  # BL: collar, hip, knee
    0.0, -1.50,  1.40,  # BR: collar, hip, knee
    0.0, -1.60,  1.70,  # FL: collar, hip, knee
   -0.10, -1.60,  1.70,  # FR: collar, hip, knee
], dtype=np.float64)


# ==============================================================================
# TERMINAL INPUT HANDLER (Non-blocking keyboard reader)
# ==============================================================================
class TerminalInputHandler:
    def __init__(self, callback):
        self.callback = callback
        self.running = True
        self.old_settings = None
        self.thread = threading.Thread(target=self._read_loop, daemon=True)

    def start(self):
        if sys.stdin.isatty():
            self.old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        self.thread.start()

    def _read_loop(self):
        while self.running:
            try:
                if sys.stdin.isatty():
                    rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
                    if rlist:
                        char = sys.stdin.read(1)
                        if char:
                            self.callback(char)
                else:
                    time.sleep(0.1)
            except Exception:
                break

    def stop(self):
        self.running = False
        if self.old_settings is not None and sys.stdin.isatty():
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            except Exception:
                pass


# ==============================================================================
# MAIN TRANSITION CONTROLLER
# ==============================================================================
class SitStandController:
    def __init__(self):
        self.motors: List[Optional[RobStrideMotorController]] = [None] * P.N_JOINTS
        self.joint_pos = np.zeros(P.N_JOINTS, dtype=np.float64)
        self.joint_vel = np.zeros(P.N_JOINTS, dtype=np.float64)
        self.joint_tau = np.zeros(P.N_JOINTS, dtype=np.float64)
        self.joint_tem = np.full(P.N_JOINTS, 25.0, dtype=np.float64)

        # Target and command buffers
        self.target_pos = np.zeros(P.N_JOINTS, dtype=np.float64)
        self.cmd_pos = np.zeros(P.N_JOINTS, dtype=np.float64)
        self.start_pos = np.zeros(P.N_JOINTS, dtype=np.float64)

        # Control parameters
        self.kp = 25.0          # Nominal stiffness matching parameters.py (25.0)
        self.kd = 1.5           # Damping
        self.duration = 4.0     # Smooth 4-second transition duration
        self.control_dt = 0.02  # 50 Hz control loop (20 ms)

        # State machine: "PASSIVE", "SIT", "STAND", "TRANSITIONING"
        self.state = "PASSIVE"
        self.target_state = "PASSIVE"
        self.is_passive = True
        self.transition_start_time = 0.0
        self.transition_progress = 0.0
        self.transition_finish_time = 0.0

        self.running = True
        self.status_msg = "Sistem siap. Motor dalam mode PASIF (Zero Torque)."
        self.lock = threading.RLock()

        # Direct Bluetooth/USB Gamepad Reader
        self.gamepad_reader = LinuxGamepadReader(callback=self._on_gamepad_edge)

        # ROS 2 Bridge Initialization
        self.has_ros = False
        self.imu_gravity = np.array([0.0, 0.0, -1.0], dtype=np.float32)
        self.imu_count = 0
        self._init_ros()

        atexit.register(self.shutdown)

        # Initialize CAN motors
        self._init_motors()

        # Start control thread (50 Hz)
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.control_thread.start()

    def _init_ros(self):
        try:
            import rclpy
            from sensor_msgs.msg import JointState, Imu, Joy
            if not rclpy.ok():
                rclpy.init()
            self.ros_node = rclpy.create_node("sit_stand_tester")
            self.joint_pub = self.ros_node.create_publisher(JointState, "/joint_states", 10)
            self.imu_sub = self.ros_node.create_subscription(Imu, "/Imu_data", self._imu_cb, 10)
            self.joy_sub = self.ros_node.create_subscription(Joy, "/joy", self._ros_joy_cb, 10)
            self.joy_remote_sub = self.ros_node.create_subscription(Joy, "/joy_remote", self._ros_joy_cb, 10)
            self.has_ros = True
            self.ros_thread = threading.Thread(target=self._ros_spin, daemon=True)
            self.ros_thread.start()
        except Exception as e:
            print(f"[WARN] ROS 2 tidak aktif / tidak tersedia: {e}")

    def _ros_spin(self):
        import rclpy
        while self.running and rclpy.ok():
            try:
                rclpy.spin_once(self.ros_node, timeout_sec=0.02)
            except Exception:
                break

    def _ros_joy_cb(self, msg):
        """Fallback callback for ROS 2 Joy topic if active."""
        if len(msg.buttons) > 0 and msg.buttons[0] == 1:
            self.start_transition("STAND", STAND_POSE)
            self.status_msg = "🎮 [ROS Joy 0] Memulai transisi BERDIRI (STANDUP)..."
        elif len(msg.buttons) > 1 and msg.buttons[1] == 1:
            # Button 1 / B
            pass
        elif len(msg.buttons) > 2 and msg.buttons[2] == 1:
            self.start_transition("SIT", SIT_POSE)
            self.status_msg = "🎮 [ROS Joy 2] Memulai transisi DUDUK (SIT 0.0 rad)..."

    def _on_gamepad_edge(self, state: XboxState, edges: Dict[str, Any]):
        """Triggered immediately upon button press (rising edge 0 -> 1) from Gamepad."""
        with self.lock:
            # Button A: STANDUP
            if "btn_a" in edges:
                self.start_transition("STAND", STAND_POSE)
                self.status_msg = "🎮 [Xbox A] Memulai transisi BERDIRI (STANDUP)..."

            # Button X: SIT DOWN
            if "btn_x" in edges:
                self.start_transition("SIT", SIT_POSE)
                self.status_msg = "🎮 [Xbox X] Memulai transisi DUDUK (SIT 0.0 rad)..."

            # Button LB or Back / View: STOP / PASSIVE ZERO-TORQUE
            if "btn_lb" in edges or "btn_back" in edges:
                self.is_passive = True
                self.state = "PASSIVE"
                self.status_msg = "🛑 [Xbox LB/Back] STOP PASIF: Torsi motor dinonaktifkan."

            # Button RB: Increase Duration (+0.5s)
            if "btn_rb" in edges:
                self.duration = min(10.0, self.duration + 0.5)
                self.status_msg = f"🎮 [Xbox RB] Durasi transisi diperpanjang (+0.5s): {self.duration:.1f} s"

            # Button Y: Decrease Duration (-0.5s)
            if "btn_y" in edges:
                self.duration = max(2.0, self.duration - 0.5)
                self.status_msg = f"🎮 [Xbox Y] Durasi transisi dipercepat (-0.5s): {self.duration:.1f} s"

            # D-Pad Up: Increase Kp (+0.5)
            if "dpad_up" in edges:
                self.kp = round(min(50.0, self.kp + 0.5), 1)
                self.status_msg = f"🎮 [Xbox D-Pad Up] Gain Kp dinaikkan (+0.5): {self.kp:.1f}"

            # D-Pad Down: Decrease Kp (-0.5)
            if "dpad_down" in edges:
                self.kp = round(max(0.0, self.kp - 0.5), 1)
                self.status_msg = f"🎮 [Xbox D-Pad Down] Gain Kp diturunkan (-0.5): {self.kp:.1f}"

            # D-Pad Right: Increase Kd (+0.1)
            if "dpad_right" in edges:
                self.kd = round(min(10.0, self.kd + 0.1), 2)
                self.status_msg = f"🎮 [Xbox D-Pad Right] Gain Kd dinaikkan (+0.1): {self.kd:.2f}"

            # D-Pad Left: Decrease Kd (-0.1)
            if "dpad_left" in edges:
                self.kd = round(max(0.0, self.kd - 0.1), 2)
                self.status_msg = f"🎮 [Xbox D-Pad Left] Gain Kd diturunkan (-0.1): {self.kd:.2f}"

    def _imu_cb(self, msg):
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        body_qx = qw
        body_qy = qz
        body_qz = -qy
        body_qw = -qx

        gx = -2.0 * (body_qx * body_qz - body_qw * body_qy)
        gy = -2.0 * (body_qy * body_qz + body_qw * body_qx)
        gz = -(1.0 - 2.0 * (body_qx * body_qx + body_qy * body_qy))

        with self.lock:
            self.imu_gravity = np.array([gx, gy, gz], dtype=np.float32)
            self.imu_count += 1

    def _init_motors(self):
        bus_list = sorted(list(set(P.DEVICE)))
        print(f"\n[INIT] Inisialisasi antarmuka CAN: {bus_list}...")

        for bus_name in bus_list:
            indices = [i for i, dev in enumerate(P.DEVICE) if dev == bus_name]
            for i in indices:
                try:
                    self.motors[i] = RobStrideMotorController(
                        bus=P.DEVICE[i],
                        motor_id=P.CAN_ID[i],
                        motor_type=P.MOTOR_TYPE[i],
                        motor_dir=P.MOTOR_DIR[i]
                    )
                    self.motors[i].enable_motor()
                    time.sleep(0.01)
                    self.motors[i].set_run_mode("CONTROL_MODE")
                    if P.MOTOR_OFFSET_ANGLE[i]:
                        self.motors[i].set_angle_offset(P.MOTOR_OFFSET_ANGLE[i])
                except Exception as e:
                    print(f"[ERROR] Gagal menghubungkan Motor #{P.CAN_ID[i]} pada {bus_name}: {e}")

        # Initial read of actual positions
        self._read_all_motors_passive()
        with self.lock:
            self.cmd_pos = self.joint_pos.copy()
            self.target_pos = self.joint_pos.copy()

    def _read_all_motors_passive(self):
        """Pure sensing read (Kp=0, Kd=0) for initial state."""
        for i in range(P.N_JOINTS):
            motor = self.motors[i]
            if motor is not None:
                try:
                    can_id, pos, vel, tau, tem = motor.send_control_command(
                        p_ref=0.0, v_ref=0.0, kp=0.0, kd=0.0, tau_ff=0.0
                    )
                    if pos is not None:
                        self.joint_pos[i] = pos
                        self.joint_vel[i] = vel
                        self.joint_tau[i] = tau
                        self.joint_tem[i] = tem
                except Exception:
                    pass

    def handle_input(self, char: str):
        c = char.upper()

        if char == '1':
            # [1] DUDUK / STANDBY
            self.start_transition("SIT", SIT_POSE)
        elif char == '2':
            # [2] STAND UP / BERDIRI
            self.start_transition("STAND", STAND_POSE)
        elif char == '3':
            # [3] WALK (DISABLED)
            with self.lock:
                self.status_msg = "⚠️ Mode [3] WALK DINONAKTIFKAN! Gunakan scripts/keyboard_teleop.py saat robot digantung."
        elif char == ' ':
            # [Space] Safe Passive Mode
            with self.lock:
                self.is_passive = True
                self.state = "PASSIVE"
                self.status_msg = "🛑 DARURAT / PASIF: Torsi motor dimatikan (Kp=0, Kd=0)."
        elif c == '+':
            with self.lock:
                self.duration = min(10.0, self.duration + 0.5)
                self.status_msg = f"Durasi transisi diubah menjadi: {self.duration:.1f} detik."
        elif c == '-':
            with self.lock:
                self.duration = max(2.0, self.duration - 0.5)
                self.status_msg = f"Durasi transisi diubah menjadi: {self.duration:.1f} detik."
        elif c == 'K':
            with self.lock:
                self.kp = round(min(50.0, self.kp + 0.5), 1)
                self.status_msg = f"Gain Kp dinaikkan (+0.5): {self.kp:.1f}"
        elif c == 'J':
            with self.lock:
                self.kp = round(max(0.0, self.kp - 0.5), 1)
                self.status_msg = f"Gain Kp diturunkan (-0.5): {self.kp:.1f}"
        elif c == 'L':
            with self.lock:
                self.kd = round(min(10.0, self.kd + 0.1), 2)
                self.status_msg = f"Gain Kd dinaikkan (+0.1): {self.kd:.2f}"
        elif c == 'H':
            with self.lock:
                self.kd = round(max(0.0, self.kd - 0.1), 2)
                self.status_msg = f"Gain Kd diturunkan (-0.1): {self.kd:.2f}"
        elif c == 'E':
            with self.lock:
                STAND_POSE[7] = min(-0.8, STAND_POSE[7] + 0.03)   # FL Hip
                STAND_POSE[10] = min(-0.8, STAND_POSE[10] + 0.03) # FR Hip
                if self.state == "STAND":
                    self.target_pos[7] = STAND_POSE[7]
                    self.target_pos[10] = STAND_POSE[10]
                self.status_msg = f"Hip Depan lebih TEGAK: {STAND_POSE[7]:.2f} rad (FL/FR)"
        elif c == 'R':
            with self.lock:
                STAND_POSE[7] = max(-2.0, STAND_POSE[7] - 0.03)   # FL Hip
                STAND_POSE[10] = max(-2.0, STAND_POSE[10] - 0.03) # FR Hip
                if self.state == "STAND":
                    self.target_pos[7] = STAND_POSE[7]
                    self.target_pos[10] = STAND_POSE[10]
                self.status_msg = f"Hip Depan lebih CONDONG: {STAND_POSE[7]:.2f} rad (FL/FR)"
        elif c == 'T':
            with self.lock:
                STAND_POSE[8] = min(2.0, STAND_POSE[8] + 0.03)    # FL Knee
                STAND_POSE[11] = min(2.0, STAND_POSE[11] + 0.03)  # FR Knee
                if self.state == "STAND":
                    self.target_pos[8] = STAND_POSE[8]
                    self.target_pos[11] = STAND_POSE[11]
                self.status_msg = f"Knee Depan dinaikkan: +{STAND_POSE[8]:.2f} rad (FL/FR)"
        elif c == 'Y':
            with self.lock:
                STAND_POSE[8] = max(1.0, STAND_POSE[8] - 0.03)    # FL Knee
                STAND_POSE[11] = max(1.0, STAND_POSE[11] - 0.03)  # FR Knee
                if self.state == "STAND":
                    self.target_pos[8] = STAND_POSE[8]
                    self.target_pos[11] = STAND_POSE[11]
                self.status_msg = f"Knee Depan diturunkan: +{STAND_POSE[8]:.2f} rad (FL/FR)"
        elif c == 'U':
            with self.lock:
                STAND_POSE[2] = min(2.0, STAND_POSE[2] + 0.03)  # BL Knee
                STAND_POSE[5] = min(2.0, STAND_POSE[5] + 0.03)  # BR Knee
                if self.state == "STAND":
                    self.target_pos[2] = STAND_POSE[2]
                    self.target_pos[5] = STAND_POSE[5]
                self.status_msg = f"Knee Belakang dinaikkan: +{STAND_POSE[2]:.2f} rad (BL/BR)"
        elif c == 'I':
            with self.lock:
                STAND_POSE[2] = max(1.0, STAND_POSE[2] - 0.03)  # BL Knee
                STAND_POSE[5] = max(1.0, STAND_POSE[5] - 0.03)  # BR Knee
                if self.state == "STAND":
                    self.target_pos[2] = STAND_POSE[2]
                    self.target_pos[5] = STAND_POSE[5]
                self.status_msg = f"Knee Belakang diturunkan: +{STAND_POSE[2]:.2f} rad (BL/BR)"
        elif c == 'O':
            with self.lock:
                STAND_POSE[1] = min(-0.8, STAND_POSE[1] + 0.03)  # BL Hip
                STAND_POSE[4] = min(-0.8, STAND_POSE[4] + 0.03)  # BR Hip
                if self.state == "STAND":
                    self.target_pos[1] = STAND_POSE[1]
                    self.target_pos[4] = STAND_POSE[4]
                self.status_msg = f"Hip Belakang lebih TEGAK: {STAND_POSE[1]:.2f} rad (BL/BR)"
        elif c == 'P':
            with self.lock:
                STAND_POSE[1] = max(-2.0, STAND_POSE[1] - 0.03)  # BL Hip
                STAND_POSE[4] = max(-2.0, STAND_POSE[4] - 0.03)  # BR Hip
                if self.state == "STAND":
                    self.target_pos[1] = STAND_POSE[1]
                    self.target_pos[4] = STAND_POSE[4]
                self.status_msg = f"Hip Belakang lebih CONDONG: {STAND_POSE[1]:.2f} rad (BL/BR)"
        elif c == 'Q':
            self.running = False

    def start_transition(self, target_state_name: str, target_pose: np.ndarray):
        with self.lock:
            # Capture actual current joint positions as starting trajectory points
            self.start_pos = self.joint_pos.copy()
            # Calculate shortest angular path to prevent 360-degree motor spinning
            diff = (target_pose - self.start_pos + np.pi) % (2 * np.pi) - np.pi
            self.target_pos = self.start_pos + diff
            self.target_state = target_state_name
            self.state = "TRANSITIONING"
            self.is_passive = False
            self.transition_start_time = time.time()
            self.transition_progress = 0.0

            action_name = "DUDUK (0.0 rad)" if target_state_name == "SIT" else f"STANDUP (Hip Belakang {STAND_POSE[1]:.2f}, Knee Belakang +{STAND_POSE[2]:.2f})"
            self.status_msg = f"Memulai transisi halus ke {action_name} [Durasi: {self.duration:.1f}s, Kp: {self.kp:.1f}]..."

    def _control_loop(self):
        """50 Hz (dt = 0.02s) continuous control and hardware CAN poll loop."""
        while self.running:
            t0 = time.time()

            with self.lock:
                is_passive = self.is_passive
                current_state = self.state
                target_state = self.target_state
                start_p = self.start_pos.copy()
                target_p = self.target_pos.copy()
                kp_val = self.kp
                kd_val = self.kd
                dur = self.duration

            if not is_passive:
                if current_state == "TRANSITIONING":
                    elapsed = time.time() - self.transition_start_time
                    progress = min(1.0, elapsed / dur)

                    # Smooth S-Curve (Cosine) Trajectory: s(t) = 0.5 * (1 - cos(pi * t))
                    # Zero start velocity, zero end velocity, no jerk!
                    s = 0.5 * (1.0 - math.cos(math.pi * progress))
                    desired_pos = start_p + s * (target_p - start_p)

                    with self.lock:
                        self.cmd_pos = desired_pos.copy()
                        self.transition_progress = progress

                    if progress >= 1.0:
                        with self.lock:
                            self.state = target_state
                            self.transition_finish_time = time.time()
                            self.status_msg = f"✅ Transisi ke {target_state} selesai secara mulus."
                elif current_state == "SIT":
                    # After 0.5s settle delay at sitting position, relax to passive zero-torque mode
                    if hasattr(self, 'transition_finish_time') and (time.time() - self.transition_finish_time >= 0.5):
                        with self.lock:
                            self.is_passive = True
                            self.state = "SIT (PASIF)"
                            self.status_msg = "✅ Posisi duduk stabil (0.5s). Motor otomatis dilemaskan (Mode PASIF)."
                else:
                    # Maintain target position
                    with self.lock:
                        self.cmd_pos = target_p.copy()

            # Send CAN commands & receive telemetry
            for i in range(P.N_JOINTS):
                motor = self.motors[i]
                if motor is not None:
                    try:
                        if is_passive:
                            # 100% Zero torque passive sensing
                            can_id, pos, vel, tau, tem = motor.send_control_command(
                                p_ref=0.0, v_ref=0.0, kp=0.0, kd=0.0, tau_ff=0.0
                            )
                        else:
                            can_id, pos, vel, tau, tem = motor.send_control_command(
                                p_ref=self.cmd_pos[i], v_ref=0.0, kp=kp_val, kd=kd_val, tau_ff=0.0
                            )

                        if pos is not None:
                            with self.lock:
                                self.joint_pos[i] = pos
                                self.joint_vel[i] = vel
                                self.joint_tau[i] = tau
                                self.joint_tem[i] = tem
                    except Exception:
                        pass

            # Publish /joint_states to ROS 2 (50 Hz)
            if self.has_ros:
                try:
                    from sensor_msgs.msg import JointState
                    with self.lock:
                        cur_pos = list(self.joint_pos)
                        cur_vel = list(self.joint_vel)
                        cur_tau = list(self.joint_tau)
                    js_msg = JointState()
                    js_msg.header.stamp = self.ros_node.get_clock().now().to_msg()
                    js_msg.name = P.JOINT_NAME.copy()
                    js_msg.position = cur_pos
                    js_msg.velocity = cur_vel
                    js_msg.effort = cur_tau
                    self.joint_pub.publish(js_msg)
                except Exception:
                    pass

            elapsed = time.time() - t0
            sleep_t = self.control_dt - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    def run_ui(self):
        input_handler = TerminalInputHandler(self.handle_input)
        input_handler.start()

        try:
            while self.running:
                gp_state = self.gamepad_reader.get_state()

                sys.stdout.write("\033[2J\033[H")  # Clear screen and move cursor to top

                with self.lock:
                    st = self.state
                    prog = self.transition_progress
                    dur = self.duration
                    kp_v = self.kp
                    kd_v = self.kd
                    status = self.status_msg
                    j_pos = self.joint_pos.copy()
                    j_cmd = self.cmd_pos.copy()
                    j_vel = self.joint_vel.copy()
                    j_tem = self.joint_tem.copy()
                    is_pass = self.is_passive
                    imu_g = self.imu_gravity.copy()
                    imu_n = self.imu_count
                    has_r = self.has_ros

                # State Header & Badge
                if is_pass:
                    state_badge = "\033[1;33m[ 🛑 MODE PASIF (ZERO TORQUE) ]\033[0m"
                elif st == "TRANSITIONING":
                    bar_len = 20
                    filled = int(prog * bar_len)
                    bar = "█" * filled + "░" * (bar_len - filled)
                    state_badge = f"\033[1;36m[ 🔄 TRANSISI: [{bar}] {prog*100:5.1f}% ({prog*dur:.1f}s / {dur:.1f}s) ]\033[0m"
                elif st == "STAND":
                    state_badge = "\033[1;32m[ 🧍 POSISI STANDUP (BERDIRI) ]\033[0m"
                elif "SIT" in st:
                    state_badge = "\033[1;34m[ 🧎 POSISI DUDUK (STANDBY 0.0 RAD) ]\033[0m"
                else:
                    state_badge = f"[ {st} ]"

                imu_status_str = f"[{imu_g[0]:+5.2f}, {imu_g[1]:+5.2f}, {imu_g[2]:+5.2f}] ({imu_n} msg)" if imu_n > 0 else "Menunggu /Imu_data"
                ros_status_str = "ROS 2 (/joint_states @ 50 Hz)" if has_r else "Direct CAN"

                # Gamepad connection string
                if gp_state.connected:
                    active_btns = gp_state.get_active_buttons_str()
                    btn_suffix = f" | Tombol: [{active_btns}]" if active_btns != "None" else ""
                    gp_status_str = f"\033[1;32m🟢 TERHUBUNG: {gp_state.name} ({gp_state.mapping_type}){btn_suffix}\033[0m"
                else:
                    gp_status_str = "\033[1;33m⚪ BELUM TERHUBUNG (Hubungkan Xbox via Bluetooth/USB)\033[0m"

                print("=" * 96)
                print(f" 🐾 NXP JAGUAR: SIT & STAND TESTER (SSH + XBOX BLUETOOTH)   {state_badge}")
                print("=" * 96)
                print(f" Status   : {status}")
                print(f" Gamepad  : {gp_status_str}")
                print(f" Setting  : Durasi = {dur:.1f} s | Kp = {kp_v:.1f} | Kd = {kd_v:.1f} | Mode: {ros_status_str}")
                print(f" Sensor   : IMU Proj Gravity = {imu_status_str}")
                print("-" * 96)
                print(f" {'ID':<4} {'Bus':<6} {'Joint Name':<18} {'Actual (rad)':<14} {'Target (rad)':<14} {'Diff (rad)':<12} {'Temp'}")
                print("-" * 96)

                for i in range(P.N_JOINTS):
                    cid = P.CAN_ID[i]
                    dev = P.DEVICE[i]
                    jname = P.JOINT_NAME[i]
                    curr = j_pos[i]
                    tgt = j_cmd[i] if not is_pass else 0.0
                    diff = curr - tgt if not is_pass else 0.0
                    tem = j_tem[i]

                    # Colorize difference
                    diff_str = f"{diff:+6.3f} rad" if not is_pass else "  --    "
                    tgt_str = f"{tgt:+6.3f} rad" if not is_pass else "  --    "
                    print(f" #{cid:<3} {dev:<6} {jname:<18} {curr:+8.4f} rad   {tgt_str:<14} {diff_str:<12} {tem:4.1f}°C")

                print("=" * 96)
                print(" 🎮 KONTROL STIK XBOX BLUETOOTH / USB:")
                print("   [A] 🧍 Stand Up (Berdiri)     | [X] 🧎 Sit Down (Duduk ke 0 rad -> Lemas)")
                print("   [LB]/[Back] 🛑 Stop Pasif     | [RB] ⏱️ Durasi (+0.5s)  |  [Y] ⏱️ Durasi (-0.5s)")
                print("   [D-Pad Up/Down] ⚡ Tune Kp (+0.5 / -0.5)  |  [D-Pad Right/Left] 🛡️ Tune Kd (+0.1 / -0.1)")
                print("-" * 96)
                print(" ⌨️  KONTROL TERMINAL SSH (Ketik tombol langsung tanpa Enter):")
                print("   [1] 🧎 DUDUK (Standby)        : Transisi halus ke posisi 0.0 rad lalu lemas")
                print("   [2] 🧍 STANDUP (Berdiri)      : Transisi halus ke posisi berdiri")
                print("   [+] / [-] : Ubah Durasi Transisi  |  [K] / [J] : Ubah Kp (±0.5)  |  [L] / [H] : Ubah Kd (±0.1)")
                print("   -- TUNING SENDI CEPAT --")
                print(f"   [E]/[R]: Hip Depan ({STAND_POSE[7]:.2f})  |  [T]/[Y]: Knee Depan (+{STAND_POSE[8]:.2f})")
                print(f"   [O]/[P]: Hip Blkg ({STAND_POSE[1]:.2f})   |  [U]/[I]: Knee Blkg (+{STAND_POSE[2]:.2f})")
                print("   [Space]   : STOP DARURAT / Mode Pasif Bebas Torsi")
                print("   [Q]       : Keluar dan Matikan Motor")
                print("=" * 96)

                time.sleep(0.1)  # 10 Hz UI refresh

        except KeyboardInterrupt:
            pass
        finally:
            input_handler.stop()
            self.gamepad_reader.stop()
            self.shutdown()

    def shutdown(self):
        self.running = False
        print("\n\nMematikan semua torsi motor secara aman...")
        for motor in self.motors:
            if motor is not None:
                try:
                    motor.send_control_command(p_ref=0.0, v_ref=0.0, kp=0.0, kd=0.0, tau_ff=0.0)
                    time.sleep(0.005)
                    motor.disable_motor()
                except Exception:
                    pass
        print("✅ Seluruh motor telah dinonaktifkan (Torque Disabled).")


def main():
    print("\nMemulai NXP Jaguar Transition Tester...")
    controller = SitStandController()
    controller.run_ui()


if __name__ == "__main__":
    main()
