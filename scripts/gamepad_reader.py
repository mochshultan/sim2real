#!/usr/bin/env python3
"""
🎮 NXP Jaguar: Universal Gamepad & Xbox Controller Reader for Linux
Supports:
- Direct Bluetooth Xbox Wireless Controller (/dev/input/js* with 15 buttons)
- Direct USB Xbox Controller (/dev/input/js* with 11 buttons)
- Auto-detection, auto-reconnection, and zero-latency unbuffered I/O
- Standalone diagnostic tool & importable module
"""

import os
import sys
import time
import glob
import fcntl
import struct
import select
import threading
from typing import Callable, Optional, Dict, Any, List

import json
import numpy as np

# IOCTL Constants for Linux Joystick API
JSIOCGNAME = lambda length: 0x80006a13 + (length << 16)
JSIOCGAXES = 0x80016a11
JSIOCGBUTTONS = 0x80016a12
DEADZONE_DEFAULT = 0.10

CALIBRATION_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "gamepad_calibration.json")

def load_gamepad_calibration() -> Optional[Dict[str, Any]]:
    if os.path.exists(CALIBRATION_FILE):
        try:
            with open(CALIBRATION_FILE, "r") as f:
                return json.load(f)
        except Exception:
            pass
    return None


class XboxState:
    """Standardized, normalized Xbox controller state."""

    def __init__(self):
        self.connected: bool = False
        self.name: str = "Tidak Terhubung"
        self.device_path: str = ""
        self.mapping_type: str = "None"  # "Bluetooth (15-btn)", "USB/xpad (11-btn)", "ROS 2"
        self.last_update_time: float = 0.0

        # Normalized Analog Axes: -1.0 to +1.0 (with zero at center)
        self.lx: float = 0.0       # Left stick horizontal (-1.0 Left .. +1.0 Right)
        self.ly: float = 0.0       # Left stick vertical (+1.0 Up .. -1.0 Down)
        self.rx: float = 0.0       # Right stick horizontal (-1.0 Left .. +1.0 Right)
        self.ry: float = 0.0       # Right stick vertical (+1.0 Up .. -1.0 Down)
        self.lt: float = 0.0       # Left Trigger (0.0 Released .. 1.0 Fully Pressed)
        self.rt: float = 0.0       # Right Trigger (0.0 Released .. 1.0 Fully Pressed)

        # Standard Buttons: 0 = Released, 1 = Pressed
        self.btn_a: int = 0
        self.btn_b: int = 0
        self.btn_x: int = 0
        self.btn_y: int = 0
        self.btn_lb: int = 0
        self.btn_rb: int = 0
        self.btn_back: int = 0      # View / Select / Share
        self.btn_start: int = 0     # Menu / Start
        self.btn_guide: int = 0     # Xbox Guide Button
        self.btn_thumb_l: int = 0   # Left Stick Click
        self.btn_thumb_r: int = 0   # Right Stick Click
        self.dpad_up: int = 0
        self.dpad_down: int = 0
        self.dpad_left: int = 0
        self.dpad_right: int = 0

        # Scaled Velocities for Robot Navigation
        self.vx: float = 0.0        # Forward / Backward velocity (m/s)
        self.vy: float = 0.0        # Lateral velocity (m/s)
        self.wz: float = 0.0        # Yaw rate (rad/s)

    def compute_velocities(self, max_vx: float = 1.0, max_vy: float = 1.0, max_wz: float = 1.0, deadzone: float = DEADZONE_DEFAULT):
        """Computes robot command velocities with auto-calibrated zero-offset and deadzone."""
        calib = load_gamepad_calibration()

        def process_axis(raw_val: float, axis_name: str) -> float:
            if calib and "axes" in calib and axis_name in calib["axes"]:
                c = calib["axes"][axis_name]
                center = c.get("center_offset", 0.0)
                dz = c.get("deadzone", deadzone)
                centered = raw_val - center
                if abs(centered) <= dz:
                    return 0.0
                sign = 1.0 if centered > 0 else -1.0
                rescaled = sign * ((abs(centered) - dz) / max(0.01, 1.0 - dz))
                return float(np.clip(rescaled, -1.0, 1.0))
            else:
                return raw_val if abs(raw_val) >= deadzone else 0.0

        # Left Stick Y -> Forward Vx (Up is +Vx)
        ly = process_axis(self.ly, "ly")
        # Left Stick X -> Lateral Vy (Left is +Vy, Right is -Vy in ROS standard)
        lx = process_axis(-self.lx, "lx")
        # Right Stick X -> Yaw Wz (Left is +Wz, Right is -Wz in ROS standard)
        rx = process_axis(-self.rx, "rx")

        self.vx = round(float(ly * max_vx), 2)
        self.vy = round(float(lx * max_vy), 2)
        self.wz = round(float(rx * max_wz), 2)

    def get_active_buttons_str(self) -> str:
        """Returns readable string of currently pressed buttons."""
        active = []
        if self.btn_a: active.append("A")
        if self.btn_b: active.append("B")
        if self.btn_x: active.append("X")
        if self.btn_y: active.append("Y")
        if self.btn_lb: active.append("LB")
        if self.btn_rb: active.append("RB")
        if self.btn_back: active.append("BACK")
        if self.btn_start: active.append("START")
        if self.btn_guide: active.append("XBOX")
        if self.btn_thumb_l: active.append("LS_CLICK")
        if self.btn_thumb_r: active.append("RS_CLICK")
        if self.dpad_up: active.append("DPAD_UP")
        if self.dpad_down: active.append("DPAD_DOWN")
        if self.dpad_left: active.append("DPAD_LEFT")
        if self.dpad_right: active.append("DPAD_RIGHT")
        return " + ".join(active) if active else "None"

    def copy(self) -> 'XboxState':
        """Deep copy of current state."""
        new_s = XboxState()
        for k, v in self.__dict__.items():
            setattr(new_s, k, v)
        return new_s


class LinuxGamepadReader:
    """
    Robust Linux Gamepad Reader with auto-discovery, auto-reconnect,
    non-blocking batch event draining, and support for both Bluetooth and USB Xbox controllers.
    """

    def __init__(self, callback: Optional[Callable[[XboxState, Dict[str, Any]], None]] = None, deadzone: float = DEADZONE_DEFAULT):
        self.callback = callback
        self.deadzone = deadzone
        self.state = XboxState()
        self.prev_state = XboxState()
        self.lock = threading.RLock()
        self.running = True

        self.thread = threading.Thread(target=self._worker_loop, daemon=True)
        self.thread.start()

    def get_state(self) -> XboxState:
        with self.lock:
            return self.state.copy()

    def is_connected(self) -> bool:
        with self.lock:
            return self.state.connected

    def stop(self):
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=0.5)

    def _worker_loop(self):
        while self.running:
            js_devices = sorted(glob.glob("/dev/input/js*"))
            if not js_devices:
                with self.lock:
                    self.state.connected = False
                    self.state.name = "Tidak Terhubung"
                    self.state.mapping_type = "None"
                time.sleep(0.5)
                continue

            dev_path = js_devices[0]
            fd = None
            try:
                fd = os.open(dev_path, os.O_RDONLY | os.O_NONBLOCK)
                
                # Query device name
                name_buf = bytearray(128)
                try:
                    fcntl.ioctl(fd, 0x80806a13, name_buf)
                    dev_name = name_buf.split(b'\x00')[0].decode('utf-8', errors='ignore').strip()
                except Exception:
                    dev_name = "Xbox Wireless Controller"

                # Query button count
                btn_buf = bytearray(1)
                fcntl.ioctl(fd, JSIOCGBUTTONS, btn_buf)
                num_buttons = btn_buf[0]

                # Query axes count
                ax_buf = bytearray(1)
                fcntl.ioctl(fd, JSIOCGAXES, ax_buf)
                num_axes = ax_buf[0]

                # Auto-determine mapping type: Bluetooth (15 btns) vs USB (11 btns)
                is_bt_map = (num_buttons >= 15)
                map_type_str = f"Bluetooth ({num_buttons}-btn)" if is_bt_map else f"USB/xpad ({num_buttons}-btn)"

                raw_axes = [0.0] * max(8, num_axes)
                raw_buttons = [0] * max(16, num_buttons)

                with self.lock:
                    self.state.connected = True
                    self.state.name = dev_name
                    self.state.device_path = dev_path
                    self.state.mapping_type = map_type_str
                    self.state.last_update_time = time.time()

                # Event loop for current open device
                while self.running:
                    rlist, _, _ = select.select([fd], [], [], 0.05)
                    if not rlist:
                        continue

                    # Drain all pending events in batch
                    events_parsed = 0
                    while True:
                        try:
                            ev_bytes = os.read(fd, 8)
                            if len(ev_bytes) < 8:
                                break
                            t, val, ev_type, num = struct.unpack("<IhBB", ev_bytes)

                            is_btn = bool(ev_type & 0x01)
                            is_axis = bool(ev_type & 0x02)

                            if is_btn and num < len(raw_buttons):
                                raw_buttons[num] = 1 if val else 0
                                events_parsed += 1
                            elif is_axis and num < len(raw_axes):
                                raw_axes[num] = val / 32767.0
                                events_parsed += 1
                        except (BlockingIOError, InterruptedError):
                            break

                    if events_parsed > 0:
                        self._update_state_from_raw(raw_axes, raw_buttons, is_bt_map)

            except Exception:
                # Device disconnected or error
                pass
            finally:
                if fd is not None:
                    try:
                        os.close(fd)
                    except Exception:
                        pass
                with self.lock:
                    self.state.connected = False
                    self.state.name = "Terputus"
                time.sleep(0.5)

    def _update_state_from_raw(self, raw_axes: List[float], raw_buttons: List[int], is_bt: bool):
        with self.lock:
            self.state.last_update_time = time.time()

            if is_bt:
                # Linux Bluetooth Xbox Mapping (15-button layout)
                self.state.btn_a = raw_buttons[0] if len(raw_buttons) > 0 else 0
                self.state.btn_b = raw_buttons[1] if len(raw_buttons) > 1 else 0
                self.state.btn_x = raw_buttons[3] if len(raw_buttons) > 3 else (raw_buttons[2] if len(raw_buttons) > 2 else 0)
                self.state.btn_y = raw_buttons[4] if len(raw_buttons) > 4 else (raw_buttons[3] if len(raw_buttons) > 3 else 0)
                self.state.btn_lb = raw_buttons[6] if len(raw_buttons) > 6 else 0
                self.state.btn_rb = raw_buttons[7] if len(raw_buttons) > 7 else 0
                self.state.btn_back = raw_buttons[10] if len(raw_buttons) > 10 else 0
                self.state.btn_start = raw_buttons[11] if len(raw_buttons) > 11 else 0
                self.state.btn_guide = raw_buttons[12] if len(raw_buttons) > 12 else 0
                self.state.btn_thumb_l = raw_buttons[13] if len(raw_buttons) > 13 else 0
                self.state.btn_thumb_r = raw_buttons[14] if len(raw_buttons) > 14 else 0

                # Axes for Bluetooth: 0:LX, 1:LY, 2:RX (ABS_Z), 3:RY (ABS_RZ), 4:LT, 5:RT, 6:DpadX, 7:DpadY
                self.state.lx = raw_axes[0] if len(raw_axes) > 0 else 0.0
                self.state.ly = -raw_axes[1] if len(raw_axes) > 1 else 0.0  # Up is +
                self.state.rx = raw_axes[2] if len(raw_axes) > 2 else 0.0
                self.state.ry = -raw_axes[3] if len(raw_axes) > 3 else 0.0  # Up is +

                # Triggers: -1.0 (released) .. +1.0 (pressed) -> map to 0.0 .. 1.0
                if len(raw_axes) > 4:
                    self.state.lt = max(0.0, (raw_axes[4] + 1.0) / 2.0)
                if len(raw_axes) > 5:
                    self.state.rt = max(0.0, (raw_axes[5] + 1.0) / 2.0)

                # D-Pad from Hat axes
                if len(raw_axes) > 6:
                    self.state.dpad_left = 1 if raw_axes[6] < -0.5 else 0
                    self.state.dpad_right = 1 if raw_axes[6] > 0.5 else 0
                if len(raw_axes) > 7:
                    self.state.dpad_up = 1 if raw_axes[7] < -0.5 else 0
                    self.state.dpad_down = 1 if raw_axes[7] > 0.5 else 0

            else:
                # Linux USB xpad Mapping (11-button layout)
                self.state.btn_a = raw_buttons[0] if len(raw_buttons) > 0 else 0
                self.state.btn_b = raw_buttons[1] if len(raw_buttons) > 1 else 0
                self.state.btn_x = raw_buttons[2] if len(raw_buttons) > 2 else 0
                self.state.btn_y = raw_buttons[3] if len(raw_buttons) > 3 else 0
                self.state.btn_lb = raw_buttons[4] if len(raw_buttons) > 4 else 0
                self.state.btn_rb = raw_buttons[5] if len(raw_buttons) > 5 else 0
                self.state.btn_back = raw_buttons[6] if len(raw_buttons) > 6 else 0
                self.state.btn_start = raw_buttons[7] if len(raw_buttons) > 7 else 0
                self.state.btn_guide = raw_buttons[8] if len(raw_buttons) > 8 else 0
                self.state.btn_thumb_l = raw_buttons[9] if len(raw_buttons) > 9 else 0
                self.state.btn_thumb_r = raw_buttons[10] if len(raw_buttons) > 10 else 0

                # Axes for USB xpad: 0:LX, 1:LY, 2:LT, 3:RX, 4:RY, 5:RT, 6:DpadX, 7:DpadY
                self.state.lx = raw_axes[0] if len(raw_axes) > 0 else 0.0
                self.state.ly = -raw_axes[1] if len(raw_axes) > 1 else 0.0
                self.state.rx = raw_axes[3] if len(raw_axes) > 3 else 0.0
                self.state.ry = -raw_axes[4] if len(raw_axes) > 4 else 0.0

                if len(raw_axes) > 2:
                    self.state.lt = max(0.0, (raw_axes[2] + 1.0) / 2.0)
                if len(raw_axes) > 5:
                    self.state.rt = max(0.0, (raw_axes[5] + 1.0) / 2.0)

                if len(raw_axes) > 6:
                    self.state.dpad_left = 1 if raw_axes[6] < -0.5 else 0
                    self.state.dpad_right = 1 if raw_axes[6] > 0.5 else 0
                if len(raw_axes) > 7:
                    self.state.dpad_up = 1 if raw_axes[7] < -0.5 else 0
                    self.state.dpad_down = 1 if raw_axes[7] > 0.5 else 0

            # Compute velocities
            self.state.compute_velocities(deadzone=self.deadzone)

            # Detect rising edges (button presses 0 -> 1)
            rising_edges = {}
            for btn_name in ["btn_a", "btn_b", "btn_x", "btn_y", "btn_lb", "btn_rb",
                             "btn_back", "btn_start", "btn_guide", "btn_thumb_l", "btn_thumb_r",
                             "dpad_up", "dpad_down", "dpad_left", "dpad_right"]:
                curr_val = getattr(self.state, btn_name)
                prev_val = getattr(self.prev_state, btn_name)
                if curr_val == 1 and prev_val == 0:
                    rising_edges[btn_name] = True

            self.prev_state = self.state.copy()

            # Callback invocation
            if self.callback is not None:
                try:
                    self.callback(self.state.copy(), rising_edges)
                except Exception as e:
                    pass


def main():
    """Interactive Gamepad Tester & Diagnostic Utility."""
    print("\n🔍 Memulai Gamepad Diagnostic Tool...")
    print("Mencari controller / stik Xbox di /dev/input/js*...\n")

    def on_gamepad_event(state: XboxState, edges: Dict[str, Any]):
        if edges:
            edge_names = ", ".join(edges.keys())

    reader = LinuxGamepadReader(callback=on_gamepad_event)

    try:
        while True:
            state = reader.get_state()
            sys.stdout.write("\033[2J\033[H")
            print("=" * 76)
            print(" 🎮 NXP JAGUAR: XBOX GAMEPAD DIAGNOSTIC TOOL")
            print("=" * 76)
            if state.connected:
                status_str = f"\033[1;32m🟢 TERHUBUNG: {state.name} ({state.mapping_type})\033[0m"
            else:
                status_str = "\033[1;33m⚪ TIDAK TERHUBUNG (Pastikan Xbox tersambung via Bluetooth/USB)\033[0m"
            print(f" Status Controller : {status_str}")
            print(f" Device Path       : {state.device_path or 'None'}")
            print("-" * 76)
            print(" 🕹️  ANALOG STICKS & TRIGGERS:")
            print(f"   ▶ Left Stick  : X = {state.lx:+5.2f} | Y = {state.ly:+5.2f} (Vx={state.vx:+5.2f}, Vy={state.vy:+5.2f} m/s)")
            print(f"   ▶ Right Stick : X = {state.rx:+5.2f} | Y = {state.ry:+5.2f} (Wz={state.wz:+5.2f} rad/s)")
            print(f"   ▶ Triggers    : LT = {state.lt:4.2f} | RT = {state.rt:4.2f}")
            print("-" * 76)
            print(" 🔘 TOMBOL AKTIF DITEKAN:")
            print(f"   {state.get_active_buttons_str()}")
            print("-" * 76)
            print(" 📋 PANDUAN PENGUJIAN TOMBOL:")
            print("   [A] : Stand Up (Berdiri)")
            print("   [B] : Walk (Jalan)")
            print("   [X] : Sit Down (Duduk ke 0 rad)")
            print("   [Y] : Stop Kecepatan / Kp Tuning")
            print("   [LB]/[Back] : Stop Darurat / Mode Pasif")
            print("   [RB] : Kp (+2.0)")
            print("=" * 76)
            print(" Tekan [Ctrl+C] untuk keluar.")
            time.sleep(0.08)
    except KeyboardInterrupt:
        print("\nMenutup Gamepad Diagnostic...")
    finally:
        reader.stop()


if __name__ == "__main__":
    main()
