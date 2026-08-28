#!/usr/bin/env python3
"""
🎮 NXP Jaguar: Xbox & Gamepad Auto-Calibration & Noise Diagnostic Tool
---------------------------------------------------------------------
Features:
1. Live noise and drift analysis for each analog axis (LX, LY, RX, RY, LT, RT).
2. Auto Zero-Calibration: Samples resting stick values to compute exact center offsets and noise margin.
3. Range Calibration: Captures full stick deflections (min/max).
4. Live Real-Time Visualizer with ASCII gauges and deadzone indicator.
5. Saves calibration profile to 'gamepad_calibration.json' for automatic loading by controller nodes.
"""

import os
import sys
import time
import glob
import json
import struct
import fcntl
import select
import numpy as np
from typing import Dict, List, Any, Optional

# ANSI Color Codes
C_RESET   = "\033[0m"
C_BOLD    = "\033[1m"
C_RED     = "\033[1;31m"
C_GREEN   = "\033[1;32m"
C_YELLOW  = "\033[1;33m"
C_BLUE    = "\033[1;34m"
C_MAGENTA = "\033[1;35m"
C_CYAN    = "\033[1;36m"
C_WHITE   = "\033[1;37m"
C_BG_RED  = "\033[41;1;37m"
C_BG_GRN  = "\033[42;1;30m"
C_CLEAR   = "\033[2J\033[H"

# IOCTL Constants for Linux Joystick API
JSIOCGNAME = lambda length: 0x80006a13 + (length << 16)
JSIOCGAXES = 0x80016a11
JSIOCGBUTTONS = 0x80016a12

CALIBRATION_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "gamepad_calibration.json")


class XboxCalibrator:
    def __init__(self):
        self.dev_path: Optional[str] = None
        self.dev_name: str = "Tidak Terdeteksi"
        self.num_buttons: int = 0
        self.num_axes: int = 0
        self.is_bluetooth: bool = False
        self.fd: Optional[int] = None
        
        # Raw axis tracking
        self.raw_axes: List[float] = [0.0] * 8
        self.raw_buttons: List[int] = [0] * 16

        # Calibration Data Structure
        self.calib_data: Dict[str, Any] = {
            "device_name": "Unknown",
            "connection_type": "Unknown",
            "timestamp": "",
            "axes": {
                "lx": {"center_offset": 0.0, "noise_std": 0.0, "deadzone": 0.12, "min_val": -1.0, "max_val": 1.0},
                "ly": {"center_offset": 0.0, "noise_std": 0.0, "deadzone": 0.12, "min_val": -1.0, "max_val": 1.0},
                "rx": {"center_offset": 0.0, "noise_std": 0.0, "deadzone": 0.12, "min_val": -1.0, "max_val": 1.0},
                "ry": {"center_offset": 0.0, "noise_std": 0.0, "deadzone": 0.12, "min_val": -1.0, "max_val": 1.0},
            }
        }

    def connect(self) -> bool:
        """Finds and opens the first available /dev/input/js* device."""
        js_devices = sorted(glob.glob("/dev/input/js*"))
        if not js_devices:
            return False

        self.dev_path = js_devices[0]
        try:
            self.fd = os.open(self.dev_path, os.O_RDONLY | os.O_NONBLOCK)
            
            # Query device name
            name_buf = bytearray(128)
            try:
                fcntl.ioctl(self.fd, 0x80806a13, name_buf)
                self.dev_name = name_buf.split(b'\x00')[0].decode('utf-8', errors='ignore').strip()
            except Exception:
                self.dev_name = "Xbox Wireless Controller"

            # Query button and axis count
            btn_buf = bytearray(1)
            fcntl.ioctl(self.fd, JSIOCGBUTTONS, btn_buf)
            self.num_buttons = btn_buf[0]

            ax_buf = bytearray(1)
            fcntl.ioctl(self.fd, JSIOCGAXES, ax_buf)
            self.num_axes = ax_buf[0]

            self.is_bluetooth = (self.num_buttons >= 15)
            self.raw_axes = [0.0] * max(8, self.num_axes)
            self.raw_buttons = [0] * max(16, self.num_buttons)

            return True
        except Exception as e:
            print(f"{C_RED}[ERROR] Gagal membuka {self.dev_path}: {e}{C_RESET}")
            return False

    def close(self):
        if self.fd is not None:
            try:
                os.close(self.fd)
            except Exception:
                pass
            self.fd = None

    def poll_events(self) -> bool:
        """Drains pending Linux Joystick events."""
        if self.fd is None:
            return False

        rlist, _, _ = select.select([self.fd], [], [], 0.005)
        if not rlist:
            return False

        has_data = False
        while True:
            try:
                ev_bytes = os.read(self.fd, 8)
                if len(ev_bytes) < 8:
                    break
                t, val, ev_type, num = struct.unpack("<IhBB", ev_bytes)
                is_btn = bool(ev_type & 0x01)
                is_axis = bool(ev_type & 0x02)

                if is_btn and num < len(self.raw_buttons):
                    self.raw_buttons[num] = 1 if val else 0
                    has_data = True
                elif is_axis and num < len(self.raw_axes):
                    self.raw_axes[num] = val / 32767.0
                    has_data = True
            except (BlockingIOError, InterruptedError):
                break
        return has_data

    def get_canonical_axes(self) -> Dict[str, float]:
        """Maps raw hardware axis indices to canonical stick labels."""
        if self.is_bluetooth:
            # Bluetooth Xbox mapping: 0:LX, 1:LY, 2:RX (ABS_Z), 3:RY (ABS_RZ)
            lx = self.raw_axes[0] if len(self.raw_axes) > 0 else 0.0
            ly = -self.raw_axes[1] if len(self.raw_axes) > 1 else 0.0  # Up is +
            rx = self.raw_axes[2] if len(self.raw_axes) > 2 else 0.0
            ry = -self.raw_axes[3] if len(self.raw_axes) > 3 else 0.0  # Up is +
        else:
            # USB xpad mapping: 0:LX, 1:LY, 3:RX, 4:RY
            lx = self.raw_axes[0] if len(self.raw_axes) > 0 else 0.0
            ly = -self.raw_axes[1] if len(self.raw_axes) > 1 else 0.0  # Up is +
            rx = self.raw_axes[3] if len(self.raw_axes) > 3 else 0.0
            ry = -self.raw_axes[4] if len(self.raw_axes) > 4 else 0.0  # Up is +

        return {"lx": lx, "ly": ly, "rx": rx, "ry": ry}

    def run_auto_calibration(self, duration_sec: float = 3.5):
        """Samples resting stick positions to compute offsets and noise thresholds."""
        print(f"{C_CLEAR}{C_BOLD}{C_CYAN}========================================================================{C_RESET}")
        print(f"{C_BOLD}{C_WHITE}  🎮 NXP JAGUAR: KALIBRASI OTOMATIS STIK XBOX (ZERO / CENTER OFFSET){C_RESET}")
        print(f"{C_BOLD}{C_CYAN}========================================================================{C_RESET}")
        print(f" Controller Terdeteksi : {C_GREEN}🟢 {self.dev_name}{C_RESET}")
        print(f" Tipe Sambungan        : {C_YELLOW}{'Bluetooth (15 Tombol)' if self.is_bluetooth else 'USB xpad (11 Tombol)'}{C_RESET}")
        print(f" Device Path           : {self.dev_path}\n")
        print(f"{C_BOLD}{C_RED}⚠️  PENTING: LEPASKAN SEMUA STIK ANALOG SEKARANG!{C_RESET}")
        print(f"{C_WHITE}   Jangan sentuh stik analog agar program dapat mengukur error resting center.{C_RESET}\n")

        for countdown in range(3, 0, -1):
            sys.stdout.write(f"\r Memulai pengambilan sampel dalam {C_BOLD}{countdown}{C_RESET} detik... ")
            sys.stdout.flush()
            time.sleep(1.0)
        print("\n\n" + f"{C_GREEN}▶ Sedang merekam noise & drift posisi tengah...{C_RESET}")

        samples = {"lx": [], "ly": [], "rx": [], "ry": []}
        t_start = time.time()
        sample_count = 0

        while (time.time() - t_start) < duration_sec:
            self.poll_events()
            axes = self.get_canonical_axes()
            for k in samples:
                samples[k].append(axes[k])
            sample_count += 1
            elapsed = time.time() - t_start
            pct = int((elapsed / duration_sec) * 100)
            bar = "█" * (pct // 5) + "░" * (20 - (pct // 5))
            sys.stdout.write(f"\r   [{C_CYAN}{bar}{C_RESET}] {pct}% | Sampel: {sample_count} data")
            sys.stdout.flush()
            time.sleep(0.01)

        print("\n\n" + f"{C_BOLD}{C_GREEN}✅ Perekaman Data Selesai! Menghitung parameter kalibrasi...{C_RESET}\n")

        # Process statistics
        results = {}
        for k in ["lx", "ly", "rx", "ry"]:
            arr = np.array(samples[k], dtype=np.float32)
            mean_val = float(np.mean(arr))
            std_val = float(np.std(arr))
            min_val = float(np.min(arr))
            max_val = float(np.max(arr))
            peak_noise = float(np.max(np.abs(arr - mean_val)))

            # Recommended deadzone: Max center deviation + 3*std + 0.05 safety buffer
            rec_deadzone = max(0.10, float(round(abs(mean_val) + 3.0 * std_val + 0.05, 2)))

            results[k] = {
                "center_offset": round(mean_val, 4),
                "noise_std": round(std_val, 4),
                "peak_noise": round(peak_noise, 4),
                "deadzone": rec_deadzone,
                "min_val": -1.0,
                "max_val": 1.0,
            }

        self.calib_data["device_name"] = self.dev_name
        self.calib_data["connection_type"] = "Bluetooth" if self.is_bluetooth else "USB"
        self.calib_data["timestamp"] = time.strftime("%Y-%m-%d %H:%M:%S")
        self.calib_data["axes"] = results

        self._print_calibration_table(results)

    def _print_calibration_table(self, res: Dict[str, Any]):
        print(f"{C_BOLD}{C_WHITE}------------------------------------------------------------------------{C_RESET}")
        print(f"{C_BOLD}{'Sumbu Stik':<18} | {'Center Bias':<14} | {'Noise (Std)':<14} | {'Deadzone Optimal':<16}{C_RESET}")
        print(f"{C_BOLD}{C_WHITE}------------------------------------------------------------------------{C_RESET}")

        axis_labels = {
            "lx": "Left Stick X (Vy)",
            "ly": "Left Stick Y (Vx)",
            "rx": "Right Stick X (Wz)",
            "ry": "Right Stick Y (Pitch)",
        }

        for k, name in axis_labels.items():
            r = res[k]
            bias_str = f"{r['center_offset']:+6.4f}"
            if abs(r['center_offset']) > 0.08:
                bias_color = C_RED
            elif abs(r['center_offset']) > 0.03:
                bias_color = C_YELLOW
            else:
                bias_color = C_GREEN

            print(f" {C_CYAN}{name:<18}{C_RESET} | {bias_color}{bias_str:<14}{C_RESET} | {r['noise_std']:<14.4f} | {C_BOLD}{C_GREEN}{r['deadzone']:<16.2f}{C_RESET}")

        print(f"{C_BOLD}{C_WHITE}------------------------------------------------------------------------{C_RESET}\n")

    def save_calibration(self, filepath: str = CALIBRATION_FILE):
        """Saves calibration dictionary to JSON file."""
        try:
            with open(filepath, "w") as f:
                json.dump(self.calib_data, f, indent=4)
            print(f"{C_GREEN}💾 Profil kalibrasi tersimpan di:{C_RESET} {C_BOLD}{filepath}{C_RESET}\n")
        except Exception as e:
            print(f"{C_RED}[ERROR] Gagal menyimpan kalibrasi ke {filepath}: {e}{C_RESET}\n")

    def run_live_visualizer(self):
        """Live ASCII gauge test with calibrated zero deadzone."""
        print(f"{C_BOLD}{C_CYAN}========================================================================{C_RESET}")
        print(f"{C_BOLD}{C_WHITE}  🎮 UJI COBA REAL-TIME DENGAN KALIBRASI AKTIF (Tekan Ctrl+C untuk Selesai){C_RESET}")
        print(f"{C_BOLD}{C_CYAN}========================================================================{C_RESET}")
        time.sleep(1.0)

        def make_gauge(val: float, deadzone: float) -> str:
            # Val ranges -1.0 to +1.0 -> Bar length 21 (-10 .. 0 .. +10)
            clamped = max(-1.0, min(1.0, val))
            pos = int(clamped * 10)  # -10 to +10
            bar = ["-"] * 21
            bar[10] = "|"
            if abs(val) < 0.001:
                status = f"{C_GREEN}[ ZERO / DIAM ]{C_RESET}"
                idx = 10 + pos
                bar[idx] = f"{C_GREEN}●{C_RESET}"
            else:
                status = f"{C_YELLOW}[ AKTIF GERAK ]{C_RESET}"
                idx = 10 + pos
                bar[idx] = f"{C_RED}█{C_RESET}"
            gauge_str = "".join(bar)
            return f"[{gauge_str}] {val:+5.2f} {status}"

        try:
            while True:
                self.poll_events()
                raw = self.get_canonical_axes()

                # Compute calibrated outputs
                calib = {}
                for k in ["lx", "ly", "rx", "ry"]:
                    c_info = self.calib_data["axes"][k]
                    raw_val = raw[k]
                    # Subtract resting center bias
                    centered = raw_val - c_info["center_offset"]
                    # Apply deadzone
                    if abs(centered) <= c_info["deadzone"]:
                        final_val = 0.0
                    else:
                        sign = 1.0 if centered > 0 else -1.0
                        # Re-scale from deadzone edge to 1.0
                        final_val = sign * ((abs(centered) - c_info["deadzone"]) / (1.0 - c_info["deadzone"]))
                        final_val = float(np.clip(final_val, -1.0, 1.0))
                    calib[k] = final_val

                sys.stdout.write(C_CLEAR)
                print(f"{C_BOLD}{C_CYAN}========================================================================{C_RESET}")
                print(f"{C_BOLD}{C_WHITE}  🎮 MONITOR REAL-TIME KALIBRASI STIK XBOX ({self.dev_name}){C_RESET}")
                print(f"{C_BOLD}{C_CYAN}========================================================================{C_RESET}")
                print(f" Status Controller : {C_GREEN}🟢 TERHUBUNG ({'Bluetooth' if self.is_bluetooth else 'USB'}){C_RESET}")
                print(f"{C_BOLD}{C_WHITE}------------------------------------------------------------------------{C_RESET}")
                print(f"{C_BOLD} 🕹️  NILAI SESUDAH KALIBRASI & DEADZONE FILTER:{C_RESET}")
                print(f"   ▶ Forward Vx (Left Y)  : {make_gauge(calib['ly'], self.calib_data['axes']['ly']['deadzone'])}")
                print(f"   ▶ Strafe Vy  (Left X)  : {make_gauge(calib['lx'], self.calib_data['axes']['lx']['deadzone'])}")
                print(f"   ▶ Yaw Rate Wz (Right X): {make_gauge(calib['rx'], self.calib_data['axes']['rx']['deadzone'])}")
                print(f"   ▶ Pitch (Right Y)      : {make_gauge(calib['ry'], self.calib_data['axes']['ry']['deadzone'])}")
                print(f"{C_BOLD}{C_WHITE}------------------------------------------------------------------------{C_RESET}")
                print(f"{C_BOLD} 📊 NILAI RAW HARDWARE (SEBELUM KALIBRASI):{C_RESET}")
                print(f"   • Raw Left Y  : {raw['ly']:+6.4f} | Bias: {self.calib_data['axes']['ly']['center_offset']:+6.4f} | DZ: {self.calib_data['axes']['ly']['deadzone']:.2f}")
                print(f"   • Raw Left X  : {raw['lx']:+6.4f} | Bias: {self.calib_data['axes']['lx']['center_offset']:+6.4f} | DZ: {self.calib_data['axes']['lx']['deadzone']:.2f}")
                print(f"   • Raw Right X : {raw['rx']:+6.4f} | Bias: {self.calib_data['axes']['rx']['center_offset']:+6.4f} | DZ: {self.calib_data['axes']['rx']['deadzone']:.2f}")
                print(f"   • Raw Right Y : {raw['ry']:+6.4f} | Bias: {self.calib_data['axes']['ry']['center_offset']:+6.4f} | DZ: {self.calib_data['axes']['ry']['deadzone']:.2f}")
                print(f"{C_BOLD}{C_WHITE}========================================================================{C_RESET}")
                print(f"{C_YELLOW} Tekan [Ctrl+C] untuk keluar.{C_RESET}")
                sys.stdout.flush()
                time.sleep(0.04)

        except KeyboardInterrupt:
            print("\n\nKeluar dari visualizer.")


def main():
    calib = XboxCalibrator()
    if not calib.connect():
        print(f"{C_RED}[ERROR] Tidak ada Gamepad Xbox terdeteksi di /dev/input/js*!{C_RESET}")
        print("Pastikan stik Xbox sudah tersambung via Bluetooth atau kabel USB.")
        sys.exit(1)

    try:
        # Phase 1: Auto Calibration
        calib.run_auto_calibration(duration_sec=3.5)

        # Phase 2: Save to JSON
        calib.save_calibration()

        # Phase 3: Interactive Live Visualizer
        print(f"{C_CYAN}Membuka Visualizer Real-Time dalam 2 detik...{C_RESET}")
        time.sleep(2.0)
        calib.run_live_visualizer()

    finally:
        calib.close()


if __name__ == "__main__":
    main()
