#!/usr/bin/env python3
"""
🎮 NXP Jaguar: Remote Xbox / Gamepad UDP Forwarder (Run on Remote PC)

Run this lightweight script on your REMOTE PC (where the Xbox controller is plugged in via USB).
It captures analog sticks and button presses, then streams them over UDP WiFi/LAN to the Robot PC.

Usage:
  pip install pygame   # (if not already installed)
  python3 remote_xbox_forwarder.py --robot-ip <ROBOT_IP_ADDRESS>

Example:
  python3 remote_xbox_forwarder.py --robot-ip 192.168.1.100
"""

import sys
import time
import socket
import json
import argparse

# ANSI Colors
C_RESET   = "\033[0m"
C_BOLD    = "\033[1m"
C_GREEN   = "\033[1;32m"
C_YELLOW  = "\033[1;33m"
C_CYAN    = "\033[1;36m"
C_RED     = "\033[1;31m"
C_WHITE   = "\033[1;37m"
C_CLEAR   = "\033[2J\033[H"

DEFAULT_PORT = 9876
DEADZONE = 0.20


def main():
    parser = argparse.ArgumentParser(description="NXP Jaguar Remote Xbox Controller UDP Forwarder")
    parser.add_argument("--robot-ip", "-i", type=str, required=True, help="IP address of the Robot PC (e.g. 192.168.1.50)")
    parser.add_argument("--port", "-p", type=int, default=DEFAULT_PORT, help=f"UDP Port (default: {DEFAULT_PORT})")
    parser.add_argument("--rate", "-r", type=float, default=50.0, help="Stream rate in Hz (default: 50 Hz)")
    args = parser.parse_args()

    robot_ip = args.robot_ip
    port = args.port
    rate_hz = args.rate
    dt = 1.0 / rate_hz

    # Initialize Pygame Joystick
    try:
        import pygame
    except ImportError:
        print(f"{C_RED}[ERROR] Library 'pygame' belum terinstall.{C_RESET}")
        print("Silakan install dengan menjalankan:")
        print("    pip install pygame")
        sys.exit(1)

    pygame.init()
    pygame.joystick.init()

    joystick_count = pygame.joystick.get_count()
    if joystick_count == 0:
        print(f"{C_YELLOW}[WARNING] Tidak ada Gamepad / Stik Xbox terdeteksi di Remote PC!{C_RESET}")
        print("Colokkan stik Xbox via USB, lalu jalankan ulang script ini.")
        sys.exit(1)

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    js_name = joystick.get_name()

    # Create UDP Socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target_addr = (robot_ip, port)

    print(f"{C_GREEN}✅ Gamepad Terhubung: {js_name}{C_RESET}")
    print(f"📡 Mengirim data ke Robot PC di: {robot_ip}:{port} @ {rate_hz:.0f} Hz\n")
    time.sleep(1.0)

    packet_count = 0
    t0_stat = time.time()
    current_fps = 0.0

    try:
        while True:
            t_start = time.time()
            pygame.event.pump()

            num_axes = joystick.get_numaxes()
            num_buttons = joystick.get_numbuttons()

            raw_axes = [joystick.get_axis(i) for i in range(num_axes)]
            raw_buttons = [joystick.get_button(i) for i in range(num_buttons)]

            # Xbox mapping:
            # Axis 0: Left Stick X  (-1.0 left .. +1.0 right) -> Vy (Geser Kiri/Kanan, in ROS left is +Vy, right is -Vy)
            # Axis 1: Left Stick Y  (-1.0 up   .. +1.0 down)  -> Vx (Maju/Mundur, inverted so up is +Vx)
            # Axis 3: Right Stick X (-1.0 left .. +1.0 right) -> Wz (Putar Kiri/Kanan, inverted so left is +Wz)

            ax_lx = -raw_axes[0] if num_axes > 0 else 0.0  # Left is +Vy, Right is -Vy
            ax_ly = -raw_axes[1] if num_axes > 1 else 0.0  # Up is +Vx, Down is -Vx
            ax_rx = -raw_axes[3] if num_axes > 3 else (-raw_axes[2] if num_axes > 2 else 0.0) # Left is +Wz

            def apply_dz(val, dz):
                if abs(val) <= dz:
                    return 0.0
                sign = 1.0 if val > 0 else -1.0
                return sign * (abs(val) - dz) / max(1e-4, 1.0 - dz)

            # Apply continuous deadzone (0.20)
            ax_lx_dz = apply_dz(ax_lx, DEADZONE)
            ax_ly_dz = apply_dz(ax_ly, DEADZONE)
            ax_rx_dz = apply_dz(ax_rx, DEADZONE)

            # Scale to velocity limits (Max 0.8 m/s, 0.5 m/s, 0.8 rad/s)
            vx = float(ax_ly_dz * 0.8)
            vy = float(ax_lx_dz * 0.5)
            wz = float(ax_rx_dz * 0.8)

            payload = {
                "type": "XBOX_GAMEPAD",
                "name": js_name,
                "axes": [round(vy, 3), round(vx, 3), round(wz, 3)] + [round(a, 3) for a in raw_axes],
                "buttons": raw_buttons,
                "vx": round(vx, 3),
                "vy": round(vy, 3),
                "wz": round(wz, 3),
                "timestamp": time.time(),
            }

            msg_bytes = json.dumps(payload).encode("utf-8")
            sock.sendto(msg_bytes, target_addr)
            packet_count += 1

            # Stats calculation
            if time.time() - t0_stat >= 1.0:
                current_fps = packet_count / (time.time() - t0_stat)
                packet_count = 0
                t0_stat = time.time()

            # Render UI
            btn_a = " [A: STANDUP] " if len(raw_buttons) > 0 and raw_buttons[0] else ""
            btn_b = " [B: WALK] " if len(raw_buttons) > 1 and raw_buttons[1] else ""
            btn_x = " [X: SIT] " if len(raw_buttons) > 2 and raw_buttons[2] else ""
            btn_y = " [Y: STOP] " if len(raw_buttons) > 3 and raw_buttons[3] else ""
            btn_lb = " [LB: ESTOP] " if len(raw_buttons) > 4 and raw_buttons[4] else ""
            active_btn_str = f"{btn_a}{btn_b}{btn_x}{btn_y}{btn_lb}".strip() or "None"

            sys.stdout.write(C_CLEAR)
            print(f"{C_BOLD}{C_CYAN}========================================================================{C_RESET}")
            print(f"{C_BOLD}{C_WHITE}  🎮 NXP JAGUAR: REMOTE XBOX CONTROLLER SENDER (REMOTE PC){C_RESET}")
            print(f"{C_BOLD}{C_CYAN}========================================================================{C_RESET}")
            print(f" Status Controller : {C_GREEN}🟢 TERHUBUNG ({js_name}){C_RESET}")
            print(f" Target Robot PC   : {C_YELLOW}{robot_ip}:{port}{C_RESET} | Stream Rate: {C_BOLD}{current_fps:4.1f} Hz{C_RESET}")
            print(f"{C_BOLD}{C_WHITE}------------------------------------------------------------------------{C_RESET}")
            print(f"{C_BOLD} Perintah Kecepatan Terkirim:{C_RESET}")
            print(f"   ▶ Vx (Maju / Mundur)   : {C_BOLD}{vx:+5.2f} m/s{C_RESET} (Stick Kiri Vertikal)")
            print(f"   ▶ Vy (Geser Kiri / Kn) : {C_BOLD}{vy:+5.2f} m/s{C_RESET} (Stick Kiri Horizontal)")
            print(f"   ▶ Wz (Putar / Yaw)     : {C_BOLD}{wz:+5.2f} rad/s{C_RESET} (Stick Kanan Horizontal)")
            print(f" Tombol Aktif Ditekan     : {C_GREEN}{active_btn_str}{C_RESET}")
            print(f"{C_BOLD}{C_WHITE}========================================================================{C_RESET}")
            print(f"{C_YELLOW} PANDUAN TOMBOL STIK XBOX:{C_RESET}")
            print(f"   [A] : 🧍 Berdiri (Stand Up)")
            print(f"   [B] : 🐾 Jalan (Walk - Mode RL)")
            print(f"   [X] : 🧎 Duduk (Sit Down / Standby)")
            print(f"   [Y] : 🛑 Stop Kecepatan (Vx=0, Vy=0, Wz=0)")
            print(f"   [LB]/[Back] : 🚨 EMERGENCY STOP / Safe Shutdown")
            print(f"   [Ctrl+C] : Keluar")
            print(f"{C_BOLD}{C_CYAN}========================================================================{C_RESET}")
            sys.stdout.flush()

            elapsed = time.time() - t_start
            sleep_t = dt - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        print("\n\nMematikan Remote Controller Sender...")
    finally:
        sock.close()
        pygame.quit()
        print("Selesai.")


if __name__ == "__main__":
    main()
