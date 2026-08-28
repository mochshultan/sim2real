#!/usr/bin/env python3
"""
🐾 NXP Jaguar: Unified Teleoperation Hub (Keyboard + Bluetooth/USB Xbox Gamepad + ROS 2)

Supports concurrent inputs:
1. SSH Keyboard Terminal: [1] Sit, [2] Stand, [3] Walk, [W/A/S/D/Q/E] Velocity, [X] Stop, [SPACE] E-Stop
2. Direct Bluetooth & USB Xbox Gamepad (/dev/input/js*)
3. Remote PC ROS 2 Joy topics (/joy, /joy_remote)
"""

import sys
import os
import time
import select
import termios
import tty
import threading
import numpy as np
from typing import Optional, Dict, Any

# Add scripts directory to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, Imu
from std_msgs.msg import Bool, String

from gamepad_reader import LinuxGamepadReader, XboxState

# ANSI Colors
C_RESET   = "\033[0m"
C_BOLD    = "\033[1m"
C_RED     = "\033[1;31m"
C_GREEN   = "\033[1;32m"
C_YELLOW  = "\033[1;33m"
C_BLUE    = "\033[1;34m"
C_MAGENTA = "\033[1;35m"
C_CYAN    = "\033[1;36m"
C_WHITE   = "\033[1;37m"
C_CLEAR   = "\033[2J\033[H"

DEADZONE = 0.15
MAX_VX = 0.8
MIN_VX = -0.8
MAX_VY = 0.5
MAX_WZ = 0.8


class UnifiedTeleopNode(Node):
    def __init__(self):
        super().__init__("jaguar_unified_teleop")

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.joy_pub = self.create_publisher(Joy, "/joy", 10)
        self.safe_stop_pub = self.create_publisher(Bool, "/jaguar/safe_stop", 10)
        self.estop_pub = self.create_publisher(Bool, "/jaguar/emergency_stop", 10)

        # Subscribers for feedback
        self.create_subscription(String, "/jaguar/status", self._status_cb, 10)
        self.create_subscription(Imu, "/Imu_data", self._imu_cb, 10)
        self.create_subscription(Imu, "/imu/data", self._imu_cb, 10)

        # Subscriber for ROS 2 Gamepad from Remote PC (if Remote PC runs joy_node)
        self.create_subscription(Joy, "/joy_remote", self._ros_joy_cb, 10)
        self.create_subscription(Joy, "/joy_raw", self._ros_joy_cb, 10)

        # State Variables
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.current_state = "STANDBY"
        self.controller_feedback = "Menunggu status controller..."
        self.last_action = "Inisialisasi siap. Menunggu input Keyboard / Xbox."
        self.imu_rpy = [0.0, 0.0, 0.0]

        # Button trigger flags (sent as single-cycle pulse)
        self.btn_standup_pulse = False
        self.btn_walk_pulse = False
        self.btn_sit_pulse = False

        self.lock = threading.RLock()
        self.running = True

        # Initialize Direct Linux Gamepad Reader
        self.gamepad_reader = LinuxGamepadReader(callback=self._on_gamepad_event, deadzone=DEADZONE)

        # Publish loop at 20 Hz
        self.timer = self.create_timer(0.05, self._publish_loop)

        # Terminal handler
        self.old_settings = None

    def _status_cb(self, msg: String):
        with self.lock:
            self.controller_feedback = msg.data

    def _imu_cb(self, msg: Imu):
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (qw * qy - qz * qx)
        pitch = np.arcsin(np.clip(sinp, -1.0, 1.0))

        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        with self.lock:
            self.imu_rpy = [np.degrees(roll), np.degrees(pitch), np.degrees(yaw)]

    def _ros_joy_cb(self, msg: Joy):
        """Callback for standard ROS 2 Joy topic from remote nodes."""
        with self.lock:
            if len(msg.axes) >= 2:
                ly = msg.axes[1] if len(msg.axes) > 1 else 0.0
                lx = msg.axes[0] if len(msg.axes) > 0 else 0.0
                rx = msg.axes[3] if len(msg.axes) > 3 else (msg.axes[2] if len(msg.axes) > 2 else 0.0)

                if abs(ly) >= DEADZONE: self.vx = round(float(np.clip(ly * MAX_VX, MIN_VX, MAX_VX)), 2)
                if abs(lx) >= DEADZONE: self.vy = round(float(np.clip(-lx * MAX_VY, -MAX_VY, MAX_VY)), 2)
                if abs(rx) >= DEADZONE: self.wz = round(float(np.clip(-rx * MAX_WZ, -MAX_WZ, MAX_WZ)), 2)

            if len(msg.buttons) > 0 and msg.buttons[0] == 1:
                self.current_state = "BERDIRI / STANDUP"
                self.btn_standup_pulse = True
                self.last_action = f"{C_CYAN}[ROS Joy A] Transisi ke BERDIRI (STANDUP){C_RESET}"
            elif len(msg.buttons) > 1 and msg.buttons[1] == 1:
                self.current_state = "JALAN / WALK (RL)"
                self.btn_walk_pulse = True
                self.last_action = f"{C_GREEN}[ROS Joy B] Mode JALAN Aktif (RL Policy PPO){C_RESET}"
            elif len(msg.buttons) > 2 and msg.buttons[2] == 1:
                self.current_state = "DUDUK / STANDBY"
                self.btn_sit_pulse = True
                self.vx = 0.0
                self.vy = 0.0
                self.wz = 0.0
                self.last_action = f"{C_YELLOW}[ROS Joy X] Transisi ke DUDUK (STANDBY){C_RESET}"

    def _on_gamepad_event(self, state: XboxState, edges: Dict[str, Any]):
        """Triggered on any analog or digital event from LinuxGamepadReader."""
        with self.lock:
            # 1. Update velocities from analog sticks
            # Only override keyboard velocity if sticks are pushed past deadzone or if stick moved
            if abs(state.lx) >= DEADZONE or abs(state.ly) >= DEADZONE or abs(state.rx) >= DEADZONE:
                self.vx = state.vx
                self.vy = state.vy
                self.wz = state.wz

            # 2. Button Edge Triggers (0 -> 1)
            # Button A: STANDUP
            if "btn_a" in edges:
                self.current_state = "BERDIRI / STANDUP"
                self.btn_standup_pulse = True
                self.last_action = f"{C_CYAN}[Xbox A] Transisi ke BERDIRI (STANDUP){C_RESET}"

            # Button B: WALK (RL)
            if "btn_b" in edges:
                self.current_state = "JALAN / WALK (RL)"
                self.btn_walk_pulse = True
                self.last_action = f"{C_GREEN}[Xbox B] Mode JALAN Aktif (RL Policy PPO){C_RESET}"

            # Button X: SIT (STANDBY)
            if "btn_x" in edges:
                self.current_state = "DUDUK / STANDBY"
                self.btn_sit_pulse = True
                self.vx = 0.0
                self.vy = 0.0
                self.wz = 0.0
                self.last_action = f"{C_YELLOW}[Xbox X] Transisi ke DUDUK (STANDBY){C_RESET}"

            # Button Y: STOP VELOCITY (V=0)
            if "btn_y" in edges:
                self.vx = 0.0
                self.vy = 0.0
                self.wz = 0.0
                self.last_action = f"{C_YELLOW}[Xbox Y] Stop Kecepatan (Vx=0, Vy=0, Wz=0){C_RESET}"

            # Button LB or Back: EMERGENCY STOP / SAFE SHUTDOWN
            if "btn_lb" in edges or "btn_back" in edges:
                self.current_state = "SAFE SHUTDOWN"
                self.vx = 0.0
                self.vy = 0.0
                self.wz = 0.0
                self.btn_standup_pulse = False
                self.btn_walk_pulse = False
                self.btn_sit_pulse = False
                safe_msg = Bool()
                safe_msg.data = True
                self.safe_stop_pub.publish(safe_msg)
                self.last_action = f"{C_RED}[Xbox LB/Back] SAFE SHUTDOWN DIPICU! Robot kembali ke 0 rad lalu mati.{C_RESET}"

    def _publish_loop(self):
        with self.lock:
            # 1. Publish /cmd_vel
            twist = Twist()
            twist.linear.x = float(self.vx)
            twist.linear.y = float(self.vy)
            twist.angular.z = float(self.wz)
            self.cmd_vel_pub.publish(twist)

            # 2. Publish /joy
            joy = Joy()
            joy.header.stamp = self.get_clock().now().to_msg()
            # Buttons: [0: Standup, 1: Walk, 2: Sit/Estop]
            btn0 = 1 if self.btn_standup_pulse else 0
            btn1 = 1 if self.btn_walk_pulse else 0
            btn2 = 1 if self.btn_sit_pulse else 0
            joy.buttons = [btn0, btn1, btn2, 0, 0, 0, 0, 0]

            # Clear pulses after single transmission
            self.btn_standup_pulse = False
            self.btn_walk_pulse = False
            self.btn_sit_pulse = False

            # Axes: [0: left_stick_h (vy), 1: left_stick_v (vx), 2: 0, 3: right_stick_h (wz)]
            ax_vx = float(np.clip(self.vx / MAX_VX, -1.0, 1.0))
            ax_vy = float(np.clip(self.vy / MAX_VY, -1.0, 1.0))
            ax_wz = float(np.clip(self.wz / MAX_WZ, -1.0, 1.0))
            joy.axes = [ax_vy, ax_vx, 0.0, ax_wz, 0.0, 0.0]
            self.joy_pub.publish(joy)

    def handle_key(self, key: str):
        with self.lock:
            k = key.lower()
            if k == '1':
                self.current_state = "DUDUK / STANDBY"
                self.btn_sit_pulse = True
                self.vx = 0.0
                self.vy = 0.0
                self.wz = 0.0
                self.last_action = f"{C_YELLOW}[Key 1] Transisi ke DUDUK (STANDBY){C_RESET}"
            elif k == '2':
                self.current_state = "BERDIRI / STANDUP"
                self.btn_standup_pulse = True
                self.vx = 0.0
                self.vy = 0.0
                self.wz = 0.0
                self.last_action = f"{C_CYAN}[Key 2] Transisi ke BERDIRI (STANDUP){C_RESET}"
            elif k == '3':
                self.current_state = "JALAN / WALK (RL)"
                self.btn_walk_pulse = True
                self.last_action = f"{C_GREEN}[Key 3] Mode JALAN Aktif (RL Policy PPO){C_RESET}"
            elif k == 'w':
                self.vx = round(min(MAX_VX, self.vx + 0.1), 2)
                self.last_action = f"{C_CYAN}[Key W] Maju (+Vx) -> {self.vx:.2f} m/s{C_RESET}"
            elif k == 's':
                self.vx = round(max(MIN_VX, self.vx - 0.1), 2)
                self.last_action = f"{C_CYAN}[Key S] Mundur (-Vx) -> {self.vx:.2f} m/s{C_RESET}"
            elif k == 'a':
                self.vy = round(min(MAX_VY, self.vy + 0.1), 2)
                self.last_action = f"{C_CYAN}[Key A] Geser Kiri (+Vy) -> {self.vy:.2f} m/s{C_RESET}"
            elif k == 'd':
                self.vy = round(max(-MAX_VY, self.vy - 0.1), 2)
                self.last_action = f"{C_CYAN}[Key D] Geser Kanan (-Vy) -> {self.vy:.2f} m/s{C_RESET}"
            elif k == 'q':
                self.wz = round(min(MAX_WZ, self.wz + 0.2), 2)
                self.last_action = f"{C_CYAN}[Key Q] Putar Kiri (+Wz) -> {self.wz:.2f} rad/s{C_RESET}"
            elif k == 'e':
                self.wz = round(max(-MAX_WZ, self.wz - 0.2), 2)
                self.last_action = f"{C_CYAN}[Key E] Putar Kanan (-Wz) -> {self.wz:.2f} rad/s{C_RESET}"
            elif k == 'x':
                self.vx = 0.0
                self.vy = 0.0
                self.wz = 0.0
                self.last_action = f"{C_YELLOW}[Key X] Stop Kecepatan (Vx=0, Vy=0, Wz=0){C_RESET}"
            elif k == ' ':
                self.current_state = "SAFE SHUTDOWN"
                self.vx = 0.0
                self.vy = 0.0
                self.wz = 0.0
                self.btn_standup_pulse = False
                self.btn_walk_pulse = False
                self.btn_sit_pulse = False
                safe_msg = Bool()
                safe_msg.data = True
                self.safe_stop_pub.publish(safe_msg)
                self.last_action = f"{C_RED}[Key SPACE] SAFE SHUTDOWN DIPICU! Robot kembali ke 0 rad lalu mati.{C_RESET}"

    def render_ui(self):
        gp_state = self.gamepad_reader.get_state()

        with self.lock:
            state_str = self.current_state
            if "WALK" in state_str:
                state_badge = f"{C_GREEN}[ 🐾 {state_str} ]{C_RESET}"
            elif "STANDUP" in state_str or "BERDIRI" in state_str:
                state_badge = f"{C_CYAN}[ 🧍 {state_str} ]{C_RESET}"
            elif "STOP" in state_str or "SHUTDOWN" in state_str:
                state_badge = f"{C_RED}[ 🚨 {state_str} ]{C_RESET}"
            else:
                state_badge = f"{C_YELLOW}[ 🪑 {state_str} ]{C_RESET}"

            # Controller health parse
            health_badge = f"{C_WHITE}Menunggu data RL...{C_RESET}"
            if "Freq:" in self.controller_feedback and "Latency:" in self.controller_feedback:
                try:
                    parts = self.controller_feedback.split("|")
                    freq_part = [p for p in parts if "Freq:" in p][0].replace("Freq:", "").replace("Hz", "").strip()
                    lat_part = [p for p in parts if "Latency:" in p][0].replace("Latency:", "").replace("ms", "").strip()
                    freq_val = float(freq_part)
                    lat_val = float(lat_part)

                    if freq_val >= 47.0 and lat_val <= 20.0:
                        health_badge = f"{C_GREEN}🟢 OK: {freq_val:.1f} Hz (Latensi: {lat_val:.1f} ms){C_RESET}"
                    elif freq_val >= 40.0:
                        health_badge = f"{C_YELLOW}🟡 WARNING: {freq_val:.1f} Hz (Latensi: {lat_val:.1f} ms){C_RESET}"
                    else:
                        health_badge = f"{C_RED}🔴 CRITICAL LAG: {freq_val:.1f} Hz (Latensi: {lat_val:.1f} ms){C_RESET}"
                except Exception:
                    health_badge = f"{C_WHITE}{self.controller_feedback}{C_RESET}"

            # Gamepad Connection Status
            if gp_state.connected:
                active_btns = gp_state.get_active_buttons_str()
                btn_suffix = f" | Tombol: [{active_btns}]" if active_btns != "None" else ""
                gp_status = f"{C_GREEN}🟢 TERHUBUNG: {gp_state.name} ({gp_state.mapping_type}){btn_suffix}{C_RESET}"
            else:
                gp_status = f"{C_YELLOW}⚪ BELUM TERHUBUNG (Pastikan Xbox Bluetooth/USB tersambung){C_RESET}"

            output = []
            output.append(f"{C_CLEAR}{C_BOLD}{C_WHITE}========================================================================{C_RESET}")
            output.append(f"{C_BOLD}{C_CYAN}  🐾 NXP JAGUAR: TELEOP (KEYBOARD SSH + XBOX BLUETOOTH){C_RESET}")
            output.append(f"{C_BOLD}{C_WHITE}========================================================================{C_RESET}")
            output.append(f" Status Mode Robot   : {state_badge}")
            output.append(f" Xbox Bluetooth Stik : {gp_status}")
            output.append(f" Kesehatan Frekuensi : {health_badge}")
            output.append(f" Orientasi IMU (RPY) : Roll: {self.imu_rpy[0]:+5.1f}° | Pitch: {self.imu_rpy[1]:+5.1f}° | Yaw: {self.imu_rpy[2]:+5.1f}°")
            output.append(f" Perintah Terakhir   : {self.last_action}")
            output.append(f"{C_BOLD}{C_WHITE}------------------------------------------------------------------------{C_RESET}")
            output.append(f"{C_BOLD} Target Kecepatan Aktif (/cmd_vel):{C_RESET}")
            output.append(f"   ▶ Vx  (Maju/Mundur)  : {C_BOLD}{self.vx:+5.2f} m/s{C_RESET}  [-0.8 .. +1.2]")
            output.append(f"   ▶ Vy  (Geser Kiri/Kn): {C_BOLD}{self.vy:+5.2f} m/s{C_RESET}  [-0.5 .. +0.5]")
            output.append(f"   ▶ Wz  (Putar Yaw)    : {C_BOLD}{self.wz:+5.2f} rad/s{C_RESET} [-1.2 .. +1.2]")
            output.append(f"{C_BOLD}{C_WHITE}========================================================================{C_RESET}")
            output.append(f"{C_BOLD}{C_YELLOW} PANDUAN KONTROL (STIK XBOX BLUETOOTH & KEYBOARD SSH):{C_RESET}")
            output.append(f"  {C_BOLD}[Xbox X] / [1]{C_RESET} Duduk / Standby   {C_BOLD}[Xbox A] / [2]{C_RESET} Berdiri     {C_BOLD}[Xbox B] / [3]{C_RESET} Jalan (RL)")
            output.append(f"  {C_BOLD}[LeftStick Y] / [W/S]{C_RESET} Maju/Mundur     {C_BOLD}[LeftStick X] / [A/D]{C_RESET} Geser Samping")
            output.append(f"  {C_BOLD}[RightStick X] / [Q/E]{C_RESET} Putar (Yaw)")
            output.append(f"  {C_BOLD}[Xbox Y] / [X]{C_RESET} Stop Kecepatan (V=0)    {C_BOLD}[Xbox LB] / [SPACE]{C_RESET} 🚨 EMERGENCY STOP")
            output.append(f"{C_BOLD}{C_WHITE}========================================================================{C_RESET}")
            sys.stdout.write("\n".join(output) + "\n")
            sys.stdout.flush()


def run_keyboard_listener(node: UnifiedTeleopNode):
    """Background thread to read SSH terminal keys non-blocking."""
    if not sys.stdin.isatty():
        node.get_logger().warn("stdin bukan TTY terminal, mode keyboard dinonaktifkan.")
        return

    node.old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    try:
        while node.running:
            rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
            if rlist:
                key = sys.stdin.read(1)
                if key == '\x03':  # Ctrl+C
                    break
                node.handle_key(key)
                node.render_ui()
            else:
                node.render_ui()
                time.sleep(0.05)
    except Exception:
        pass
    finally:
        if node.old_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.old_settings)


def main(args=None):
    rclpy.init(args=args)
    node = UnifiedTeleopNode()

    # Start keyboard listener in background thread
    listener_thread = threading.Thread(target=run_keyboard_listener, args=(node,), daemon=True)
    listener_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.gamepad_reader.stop()
        if node.old_settings is not None and sys.stdin.isatty():
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.old_settings)
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()
        print("\nUnified Teleop Node dimatikan.")


if __name__ == "__main__":
    main()
