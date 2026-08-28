#!/usr/bin/env python3
"""
📊 NXP Jaguar: Live Real-Time Joint Telemetry & Jump Detection Monitor
----------------------------------------------------------------------
Monitors /joint_states and CAN feedback in real-time.
Logs any sudden position jumps (> 0.03 rad in 1 frame) with timestamps.
"""

import sys
import time
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

C_RESET   = "\033[0m"
C_BOLD    = "\033[1m"
C_RED     = "\033[1;31m"
C_GREEN   = "\033[1;32m"
C_YELLOW  = "\033[1;33m"
C_CYAN    = "\033[1;36m"
C_MAGENTA = "\033[1;35m"
C_WHITE   = "\033[1;37m"
C_CLEAR   = "\033[2J\033[H"

JOINT_NAMES = [
    'BL_collar', 'BL_hip', 'BL_knee',
    'BR_collar', 'BR_hip', 'BR_knee',
    'FL_collar', 'FL_hip', 'FL_knee',
    'FR_collar', 'FR_hip', 'FR_knee',
]


class JointMonitorNode(Node):
    def __init__(self):
        super().__init__('live_joint_monitor')
        self.sub = self.create_subscription(JointState, '/joint_states', self._js_cb, 10)
        self.last_pos = None
        self.last_time = time.time()
        self.jump_log = []
        self.msg_count = 0
        self.current_pos = [0.0] * 12
        self.current_vel = [0.0] * 12
        self.current_tau = [0.0] * 12
        self.hz = 0.0

    def _js_cb(self, msg: JointState):
        now = time.time()
        dt = now - self.last_time
        if dt > 0:
            self.hz = 0.9 * self.hz + 0.1 * (1.0 / dt)
        self.last_time = now
        self.msg_count += 1

        pos = list(msg.position)
        vel = list(msg.velocity) if msg.velocity else [0.0] * len(pos)
        tau = list(msg.effort) if msg.effort else [0.0] * len(pos)

        if len(pos) == 12:
            if self.last_pos is not None:
                # Detect any sudden jumps (> 0.03 rad = ~1.7 deg in 1 frame)
                for i in range(12):
                    delta = abs(pos[i] - self.last_pos[i])
                    if delta > 0.03:
                        name = msg.name[i] if (msg.name and len(msg.name) > i) else JOINT_NAMES[i]
                        t_str = time.strftime("%H:%M:%S")
                        log_entry = f"[{t_str}] ⚠️ JUMP on {name} (#{i}): {self.last_pos[i]:+.3f} -> {pos[i]:+.3f} rad (Δ={delta:.3f} rad / {math.degrees(delta):.1f}°)"
                        self.jump_log.append(log_entry)
                        if len(self.jump_log) > 6:
                            self.jump_log.pop(0)

            self.last_pos = pos.copy()
            self.current_pos = pos
            self.current_vel = vel
            self.current_tau = tau


def main():
    rclpy.init()
    node = JointMonitorNode()

    print("Live Joint Monitor started. Waiting for /joint_states...")

    last_render = 0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            t_now = time.time()
            if (t_now - last_render) >= 0.05:  # 20 Hz terminal refresh
                last_render = t_now

                sys.stdout.write(C_CLEAR)
                print(f"{C_BOLD}{C_CYAN}========================================================================{C_RESET}")
                print(f"{C_BOLD}{C_WHITE}  📊 NXP JAGUAR: LIVE JOINT TELEMETRY & JUMP DETECTOR{C_RESET}")
                print(f"{C_BOLD}{C_CYAN}========================================================================{C_RESET}")
                print(f" Status Topic `/joint_states` : {C_GREEN}🟢 TERHUBUNG ({node.hz:.1f} Hz){C_RESET} | Pesan: {node.msg_count}")
                print(f"{C_BOLD}{C_WHITE}------------------------------------------------------------------------{C_RESET}")
                print(f"{C_BOLD}{'Kaki':<6} | {'Sendi':<12} | {'Posisi (rad)':<14} | {'Posisi (deg)':<14} | {'Kecepatan':<10} | {'Torsi (Nm)':<10}{C_RESET}")
                print(f"{C_BOLD}{C_WHITE}------------------------------------------------------------------------{C_RESET}")

                legs = ["BL", "BR", "FL", "FR"]
                types = ["Collar (Coxa)", "Hip (Pitch)", "Knee (Pitch)"]
                for i in range(12):
                    leg = legs[i // 3]
                    jtype = types[i % 3]
                    p = node.current_pos[i]
                    p_deg = math.degrees(p)
                    v = node.current_vel[i]
                    t = node.current_tau[i]

                    # Color highlight for Coxa
                    is_coxa = (i % 3 == 0)
                    row_color = C_YELLOW if is_coxa else C_WHITE
                    coxa_tag = " (COXA)" if is_coxa else ""

                    print(f" {C_CYAN}{leg:<6}{C_RESET} | {row_color}{jtype:<12}{C_RESET} | {p:+8.4f} rad     | {p_deg:+7.2f}°       | {v:+6.2f}     | {t:+6.2f}")

                print(f"{C_BOLD}{C_WHITE}------------------------------------------------------------------------{C_RESET}")
                print(f"{C_BOLD} 🚨 LOG LONJAKAN / JUMP DETECTOR (Perubahan > 0.03 rad dlm 1 frame):{C_RESET}")
                if node.jump_log:
                    for l in node.jump_log:
                        print(f"   {C_RED}{l}{C_RESET}")
                else:
                    print(f"   {C_GREEN}✅ Belum terdeteksi lonjakan posisi diskrit (gerakan kontinu).{C_RESET}")
                print(f"{C_BOLD}{C_CYAN}========================================================================{C_RESET}")
                print(f"{C_YELLOW} Tekan Ctrl+C untuk berhenti.{C_RESET}")
                sys.stdout.flush()

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
