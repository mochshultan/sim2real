#!/usr/bin/env python3
"""
🐾 NXP Jaguar: Interactive Keyboard Teleoperation & State Controller (ROS 2)

Keybindings:
  [1]     : 🪑 Duduk / Standby (SITDOWN -> STANDBY)
  [2]     : 🧍 Berdiri (STANDUP - Smooth trajectory to nominal q0)
  [3]     : 🐾 Jalan (WALK - Activate Isaac Lab RL Policy)
  [W]     : Maju (+Vx)
  [S]     : Mundur (-Vx)
  [A]     : Geser Kiri (+Vy)
  [D]     : Geser Kanan (-Vy)
  [Q]     : Putar Kiri (+Wz)
  [E]     : Putar Kanan (-Wz)
  [X]     : Stop Gerakan (Vx=0, Vy=0, Wz=0)
  [SPACE] : 🚨 EMERGENCY STOP (Stop seketika & kembali Duduk)
  [Ctrl+C]: Keluar program
"""

import sys
import os
import time
import select
import termios
import tty
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, Imu
from std_msgs.msg import Bool, String

# ANSI Colors
C_RESET  = "\033[0m"
C_BOLD   = "\033[1m"
C_RED    = "\033[1;31m"
C_GREEN  = "\033[1;32m"
C_YELLOW = "\033[1;33m"
C_BLUE   = "\033[1;34m"
C_MAGENTA= "\033[1;35m"
C_CYAN   = "\033[1;36m"
C_WHITE  = "\033[1;37m"
C_CLEAR  = "\033[2J\033[H"


class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__("jaguar_keyboard_teleop")

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.joy_pub = self.create_publisher(Joy, "/joy", 10)
        self.estop_pub = self.create_publisher(Bool, "/jaguar/emergency_stop", 10)

        # Subscribers for feedback
        self.create_subscription(String, "/jaguar/status", self._status_cb, 10)
        self.create_subscription(Imu, "/Imu_data", self._imu_cb, 10)

        # State Variables
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.current_state = "STANDBY"
        self.controller_feedback = "Menunggu status controller..."
        self.last_key_action = "Inisialisasi siap."
        self.imu_rpy = [0.0, 0.0, 0.0]

        # Button trigger flags (sent as pulse)
        self.btn_standup_pulse = False
        self.btn_walk_pulse = False
        self.btn_sit_pulse = False

        self.lock = threading.Lock()
        self.running = True

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
        # Convert quaternion to approximate RPY (degrees)
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
            ax_vx = float(np.clip(self.vx / 1.0, -1.0, 1.0))
            ax_vy = float(np.clip(self.vy / 0.5, -1.0, 1.0))
            ax_wz = float(np.clip(self.wz / 1.2, -1.0, 1.0))
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
                self.last_key_action = f"{C_YELLOW}[1] Transisi ke DUDUK (STANDBY){C_RESET}"
            elif k == '2':
                self.current_state = "BERDIRI / STANDUP"
                self.btn_standup_pulse = True
                self.vx = 0.0
                self.vy = 0.0
                self.wz = 0.0
                self.last_key_action = f"{C_CYAN}[2] Transisi ke BERDIRI (STANDUP){C_RESET}"
            elif k == '3':
                self.current_state = "JALAN / WALK (RL)"
                self.btn_walk_pulse = True
                self.last_key_action = f"{C_GREEN}[3] Mode JALAN Aktif (RL Policy PPO){C_RESET}"
            elif k == 'w':
                self.vx = round(min(1.2, self.vx + 0.1), 2)
                self.last_key_action = f"{C_CYAN}[W] Maju (+Vx) -> {self.vx:.2f} m/s{C_RESET}"
            elif k == 's':
                self.vx = round(max(-0.8, self.vx - 0.1), 2)
                self.last_key_action = f"{C_CYAN}[S] Mundur (-Vx) -> {self.vx:.2f} m/s{C_RESET}"
            elif k == 'a':
                self.vy = round(min(0.5, self.vy + 0.1), 2)
                self.last_key_action = f"{C_CYAN}[A] Geser Kiri (+Vy) -> {self.vy:.2f} m/s{C_RESET}"
            elif k == 'd':
                self.vy = round(max(-0.5, self.vy - 0.1), 2)
                self.last_key_action = f"{C_CYAN}[D] Geser Kanan (-Vy) -> {self.vy:.2f} m/s{C_RESET}"
            elif k == 'q':
                self.wz = round(min(1.2, self.wz + 0.2), 2)
                self.last_key_action = f"{C_CYAN}[Q] Putar Kiri (+Wz) -> {self.wz:.2f} rad/s{C_RESET}"
            elif k == 'e':
                self.wz = round(max(-1.2, self.wz - 0.2), 2)
                self.last_key_action = f"{C_CYAN}[E] Putar Kanan (-Wz) -> {self.wz:.2f} rad/s{C_RESET}"
            elif k == 'x':
                self.vx = 0.0
                self.vy = 0.0
                self.wz = 0.0
                self.last_key_action = f"{C_YELLOW}[X] Stop Kecepatan (Vx=0, Vy=0, Wz=0){C_RESET}"
            elif k == ' ':
                self.current_state = "🚨 EMERGENCY STOP"
                self.vx = 0.0
                self.vy = 0.0
                self.wz = 0.0
                self.btn_sit_pulse = True
                estop_msg = Bool()
                estop_msg.data = True
                self.estop_pub.publish(estop_msg)
                self.last_key_action = f"{C_RED}[SPACE] 🚨 EMERGENCY STOP DIPICU! Robot Duduk.{C_RESET}"

    def render_ui(self):
        with self.lock:
            state_str = self.current_state
            if "WALK" in state_str:
                state_badge = f"{C_GREEN}[ 🐾 {state_str} ]{C_RESET}"
            elif "STANDUP" in state_str or "BERDIRI" in state_str:
                state_badge = f"{C_CYAN}[ 🧍 {state_str} ]{C_RESET}"
            elif "STOP" in state_str:
                state_badge = f"{C_RED}[ 🚨 {state_str} ]{C_RESET}"
            else:
                state_badge = f"{C_YELLOW}[ 🪑 {state_str} ]{C_RESET}"

            output = []
            output.append(f"{C_CLEAR}{C_BOLD}{C_WHITE}========================================================================{C_RESET}")
            output.append(f"{C_BOLD}{C_CYAN}  🐾 NXP JAGUAR: KEYBOARD CONTROLLER & TELEOP (ROS 2){C_RESET}")
            output.append(f"{C_BOLD}{C_WHITE}========================================================================{C_RESET}")
            output.append(f" Status Mode Robot   : {state_badge}")
            output.append(f" Feedback Controller : {C_WHITE}{self.controller_feedback}{C_RESET}")
            output.append(f" Orientasi IMU (RPY) : Roll: {self.imu_rpy[0]:+5.1f}° | Pitch: {self.imu_rpy[1]:+5.1f}° | Yaw: {self.imu_rpy[2]:+5.1f}°")
            output.append(f" Perintah Terakhir   : {self.last_key_action}")
            output.append(f"{C_BOLD}{C_WHITE}------------------------------------------------------------------------{C_RESET}")
            output.append(f"{C_BOLD} Target Kecepatan (/cmd_vel):{C_RESET}")
            output.append(f"   ▶ Vx  (Maju/Mundur)  : {C_BOLD}{self.vx:+5.2f} m/s{C_RESET}  [-0.8 .. +1.2]")
            output.append(f"   ▶ Vy  (Geser Kiri/Kn): {C_BOLD}{self.vy:+5.2f} m/s{C_RESET}  [-0.5 .. +0.5]")
            output.append(f"   ▶ Wz  (Putar Yaw)    : {C_BOLD}{self.wz:+5.2f} rad/s{C_RESET} [-1.2 .. +1.2]")
            output.append(f"{C_BOLD}{C_WHITE}========================================================================{C_RESET}")
            output.append(f"{C_BOLD}{C_YELLOW} PANDUAN TOMBOL KEYBOARD:{C_RESET}")
            output.append(f"  {C_BOLD}[1]{C_RESET} Duduk / Standby       {C_BOLD}[2]{C_RESET} Berdiri (Standup)    {C_BOLD}[3]{C_RESET} Jalan (Mode RL)")
            output.append(f"  {C_BOLD}[W]{C_RESET} Maju (+0.1 m/s)       {C_BOLD}[S]{C_RESET} Mundur (-0.1 m/s)")
            output.append(f"  {C_BOLD}[A]{C_RESET} Geser Kiri (+0.1 m/s) {C_BOLD}[D]{C_RESET} Geser Kanan (-0.1 m/s)")
            output.append(f"  {C_BOLD}[Q]{C_RESET} Putar Kiri (+0.2 rad) {C_BOLD}[E]{C_RESET} Putar Kanan (-0.2 rad)")
            output.append(f"  {C_BOLD}[X]{C_RESET} Stop Kecepatan (V=0)  {C_BOLD}[SPACE]{C_RESET} 🚨 EMERGENCY STOP (E-STOP)")
            output.append(f"  {C_BOLD}[Ctrl+C]{C_RESET} Keluar")
            output.append(f"{C_BOLD}{C_WHITE}========================================================================{C_RESET}")
            sys.stdout.write("\n".join(output) + "\n")
            sys.stdout.flush()


def run_keyboard_listener(node: KeyboardTeleopNode):
    """Background thread to read terminal keys non-blocking."""
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
                # Periodic refresh for IMU & Controller status
                node.render_ui()
                time.sleep(0.05)
    except Exception as e:
        pass
    finally:
        if node.old_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.old_settings)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleopNode()

    # Start keyboard listener in background thread
    listener_thread = threading.Thread(target=run_keyboard_listener, args=(node,), daemon=True)
    listener_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        if node.old_settings is not None and sys.stdin.isatty():
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.old_settings)
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()
        print("\nKeyboard Teleop Node dimatikan.")


if __name__ == "__main__":
    main()
