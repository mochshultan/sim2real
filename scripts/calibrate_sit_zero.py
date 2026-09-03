#!/usr/bin/env python3
"""
🐾 NXP Jaguar: Interactive True Zero & Sitting Pose Calibration Tool (CAN Direct)

Alur Kalibrasi:
1. Tekan [Enter] untuk transisi mulus ke posisi Duduk & Hold (3.0s S-curve).
2. Tekan [M] untuk TOGGLE MODE: [SINGLE (Kiri/Kanan Terpisah)] <---> [PAIRED (Kiri+Kanan Simetris)].
3. Pilih Joint yang ingin diatur dengan tombol Angka/Huruf (1..6 / Q..Y / A..J).
4. Tekan [↑] (Panah Atas) atau [↓] (Panah Bawah) / [+] / [-] untuk LANGSUNG MENGGERAKKAN MOTOR (±0.05 / ±0.01 rad).
5. Tekan [S] untuk SIMPAN & KELUAR -> Otomatis cetak offset baru.
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
from typing import Optional, List, Dict, Any, Tuple

# Add scripts directory to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import parameters as P
from robstride_motor_lib import RobStrideMotorController
from gamepad_reader import LinuxGamepadReader, XboxState

# ANSI Colors
C_RESET   = "\033[0m"
C_BOLD    = "\033[1m"
C_GREEN   = "\033[1;32m"
C_YELLOW  = "\033[1;33m"
C_CYAN    = "\033[1;36m"
C_RED     = "\033[1;31m"
C_MAGENTA = "\033[1;35m"
C_WHITE   = "\033[1;37m"
C_BG_BLUE = "\033[44;1;37m"
C_BG_CYAN = "\033[46;1;30m"
C_CLEAR   = "\033[2J\033[H"

# Joint indices in ROS CAN Order:
# BL: 0=Collar, 1=Hip, 2=Knee
# BR: 3=Collar, 4=Hip, 5=Knee
# FL: 6=Collar, 7=Hip, 8=Knee
# FR: 9=Collar, 10=Hip, 11=Knee

SINGLE_SELECTION_MAP: Dict[str, Tuple[str, List[int]]] = {
    # Front-Left (FL)
    '1': ('FL Coxa / Collar', [6]),
    'q': ('FL Coxa / Collar', [6]),
    '2': ('FL Hip',           [7]),
    'w': ('FL Hip',           [7]),
    '3': ('FL Knee',          [8]),
    'e': ('FL Knee',          [8]),

    # Front-Right (FR)
    '4': ('FR Coxa / Collar', [9]),
    'r': ('FR Coxa / Collar', [9]),
    '5': ('FR Hip',           [10]),
    't': ('FR Hip',           [10]),
    '6': ('FR Knee',          [11]),
    'y': ('FR Knee',          [11]),

    # Back-Left (BL)
    '7': ('BL Coxa / Collar', [0]),
    'a': ('BL Coxa / Collar', [0]),
    '8': ('BL Hip',           [1]),
    'd': ('BL Hip',           [1]),
    '9': ('BL Knee',          [2]),
    'f': ('BL Knee',          [2]),

    # Back-Right (BR)
    'u': ('BR Coxa / Collar', [3]),
    'g': ('BR Coxa / Collar', [3]),
    'i': ('BR Hip',           [4]),
    'h': ('BR Hip',           [4]),
    'o': ('BR Knee',          [5]),
    'j': ('BR Knee',          [5]),
}

PAIRED_SELECTION_MAP: Dict[str, Tuple[str, List[int]]] = {
    '1': ('Kedua Coxa Depan (FL + FR)',  [6, 9]),
    'q': ('Kedua Coxa Depan (FL + FR)',  [6, 9]),
    'z': ('Kedua Coxa Depan (FL + FR)',  [6, 9]),

    '2': ('Kedua Hip Depan (FL + FR)',   [7, 10]),
    'w': ('Kedua Hip Depan (FL + FR)',   [7, 10]),
    'x': ('Kedua Hip Depan (FL + FR)',   [7, 10]),

    '3': ('Kedua Knee Depan (FL + FR)',  [8, 11]),
    'e': ('Kedua Knee Depan (FL + FR)',  [8, 11]),
    'c': ('Kedua Knee Depan (FL + FR)',  [8, 11]),

    '4': ('Kedua Coxa Belakang (BL + BR)',[0, 3]),
    'r': ('Kedua Coxa Belakang (BL + BR)',[0, 3]),
    'v': ('Kedua Coxa Belakang (BL + BR)',[0, 3]),

    '5': ('Kedua Hip Belakang (BL + BR)', [1, 4]),
    't': ('Kedua Hip Belakang (BL + BR)', [1, 4]),
    'b': ('Kedua Hip Belakang (BL + BR)', [1, 4]),

    '6': ('Kedua Knee Belakang (BL + BR)',[2, 5]),
    'y': ('Kedua Knee Belakang (BL + BR)',[2, 5]),
    'n': ('Kedua Knee Belakang (BL + BR)',[2, 5]),
}


class TerminalInputHandler:
    """Robust raw-mode asynchronous terminal key reader with full multi-byte escape handling."""
    def __init__(self, callback):
        self.callback = callback
        self.running = True
        self.old_settings = None
        self.thread = threading.Thread(target=self._read_loop, daemon=True)

    def start(self):
        if sys.stdin.isatty():
            self.old_settings = termios.tcgetattr(sys.stdin)
            tty.setraw(sys.stdin.fileno())
        self.thread.start()

    def _read_loop(self):
        fd = sys.stdin.fileno() if sys.stdin.isatty() else None
        while self.running and fd is not None:
            try:
                rlist, _, _ = select.select([fd], [], [], 0.05)
                if not rlist:
                    time.sleep(0.01)
                    continue

                b = os.read(fd, 1)
                if not b:
                    continue

                if b == b'\x1b':  # Escape sequence prefix
                    seq = b''
                    while True:
                        r, _, _ = select.select([fd], [], [], 0.03)
                        if not r:
                            break
                        seq += os.read(fd, 1)
                        if len(seq) >= 5 or seq.endswith((b'A', b'B', b'C', b'D', b'~')):
                            break

                    if seq in [b'[A', b'OA'] or seq.endswith(b'A'):
                        self.callback("KEY_UP")
                    elif seq in [b'[B', b'OB'] or seq.endswith(b'B'):
                        self.callback("KEY_DOWN")
                    elif seq in [b'[C', b'OC', b'[D', b'OD']:
                        pass  # Disregard left/right arrows
                    elif not seq:
                        self.callback("ESC")
                elif b in [b'\r', b'\n']:
                    self.callback("ENTER")
                elif b == b'\x03':  # Ctrl+C
                    self.callback("CTRL_C")
                elif b == b'\t':
                    self.callback("TAB")
                elif b == b' ':
                    self.callback("SPACE")
                else:
                    try:
                        ch = b.decode('utf-8', errors='ignore')
                        if ch:
                            self.callback(ch)
                    except Exception:
                        pass
            except Exception:
                break

    def stop(self):
        self.running = False
        if self.old_settings is not None and sys.stdin.isatty():
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            except Exception:
                pass


class SitZeroCalibrator:
    def __init__(self):
        self.motors: List[Optional[RobStrideMotorController]] = [None] * P.N_JOINTS
        self.joint_pos = np.zeros(P.N_JOINTS, dtype=np.float64)
        self.joint_vel = np.zeros(P.N_JOINTS, dtype=np.float64)
        self.joint_tau = np.zeros(P.N_JOINTS, dtype=np.float64)
        self.joint_tem = np.full(P.N_JOINTS, 25.0, dtype=np.float64)

        # Baseline offset angles from parameters.py
        self.base_offsets = np.array(P.MOTOR_OFFSET_ANGLE, dtype=np.float64)
        # Real-time delta adjustments in joint frame (rad)
        self.delta_q = np.zeros(P.N_JOINTS, dtype=np.float64)
        
        # Trajectory command buffers
        self.start_pos = np.zeros(P.N_JOINTS, dtype=np.float64)
        self.cmd_pos = np.zeros(P.N_JOINTS, dtype=np.float64)

        # Control & Trajectory Parameters: Coxa Kp=18.0, Kd=1.0 | Hip & Knee Kp=25.0, Kd=1.5
        self.kp_coxa = 18.0     # Coxa stiffness
        self.kd_coxa = 1.0      # Coxa damping
        self.kp = 25.0          # Stiffness for sit holding & tuning (Hip/Knee)
        self.kd = 1.5           # Damping (Hip/Knee)
        self.step_size = 0.05   # Default adjustment step (±0.05 rad ≈ 2.86°)
        self.control_dt = 0.02  # 50 Hz control loop (20 ms)
        self.max_rate = 1.2     # Max rate of angle change (rad/s) during manual tuning
        
        # State machine: "PASSIVE", "TRANSITIONING", "HOLDING"
        self.state = "PASSIVE"
        self.is_active = False
        self.transition_start_time = 0.0
        self.transition_duration = 3.0
        self.transition_progress = 0.0
        
        # Mode: "SINGLE" or "PAIRED"
        self.mode = "SINGLE"
        self.active_key = '2'  # Default to FL Hip (Single) or Front Hips (Paired)
        self._update_selection()

        self.running = True
        self.save_requested = False
        self.status_msg = "Motor siap (PASIF). Tekan [Enter] untuk transisi mulus ke Duduk & Hold."
        self.last_key_pressed = "None"
        self.lock = threading.RLock()

        # Direct Gamepad reader
        self.gamepad_reader = LinuxGamepadReader(callback=self._on_gamepad_edge)

        atexit.register(self.shutdown)
        self._init_motors()

        # Start control thread (50 Hz)
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.control_thread.start()

    def _update_selection(self):
        """Updates selected name and indices according to current mode and key."""
        if self.mode == "PAIRED":
            if self.active_key in PAIRED_SELECTION_MAP:
                self.selected_name, self.selected_indices = PAIRED_SELECTION_MAP[self.active_key]
            else:
                self.active_key = '2'
                self.selected_name, self.selected_indices = PAIRED_SELECTION_MAP['2']
        else:
            if self.active_key in SINGLE_SELECTION_MAP:
                self.selected_name, self.selected_indices = SINGLE_SELECTION_MAP[self.active_key]
            else:
                self.active_key = '2'
                self.selected_name, self.selected_indices = SINGLE_SELECTION_MAP['2']

    def toggle_mode(self):
        """Toggles between SINGLE and PAIRED calibration modes."""
        with self.lock:
            self.mode = "PAIRED" if self.mode == "SINGLE" else "SINGLE"
            self._update_selection()
            mode_desc = f"{C_BOLD}{C_CYAN}PAIRED (Kiri+Kanan Simetris){C_RESET}" if self.mode == "PAIRED" else f"{C_BOLD}{C_GREEN}SINGLE (Kiri/Kanan Terpisah){C_RESET}"
            self.status_msg = f"🔀 Mode diubah ke: {mode_desc} | Dipilih: {self.selected_name}"

    def _init_motors(self):
        bus_list = sorted(list(set(P.DEVICE)))
        print(f"\r\n{C_CYAN}[INIT] Inisialisasi antarmuka CAN: {bus_list} & 12 Motor RobStride RS00...{C_RESET}\r\n")

        for bus_name in bus_list:
            indices = [i for i, dev in enumerate(P.DEVICE) if dev == bus_name]
            for i in indices:
                try:
                    motor = RobStrideMotorController(
                        bus=P.DEVICE[i],
                        motor_id=P.CAN_ID[i],
                        motor_type=P.MOTOR_TYPE[i],
                        motor_dir=P.MOTOR_DIR[i]
                    )
                    motor.enable_motor()
                    time.sleep(0.01)
                    motor.set_run_mode("CONTROL_MODE")
                    if P.MOTOR_OFFSET_ANGLE[i]:
                        motor.set_angle_offset(self.base_offsets[i])
                    self.motors[i] = motor
                except Exception as e:
                    print(f"\r\n{C_RED}[ERROR] Gagal menghubungkan Motor #{P.CAN_ID[i]} ({P.JOINT_NAME[i]}) pada {bus_name}: {e}{C_RESET}\r\n")

        self._read_all_motors_passive()
        with self.lock:
            self.cmd_pos = self.joint_pos.copy()
            self.start_pos = self.joint_pos.copy()

        print(f"\r\n{C_GREEN}✅ 12 Motor terhubung & terkonfigurasi dalam mode PASIF (Zero Torque).{C_RESET}\r\n")

    def _read_all_motors_passive(self):
        """Reads motor sensors in passive mode (Zero Torque)."""
        for i in range(P.N_JOINTS):
            motor = self.motors[i]
            if motor is not None:
                try:
                    _, pos, vel, tau, tem = motor.send_control_command(
                        p_ref=0.0, v_ref=0.0, kp=0.0, kd=0.0, tau_ff=0.0
                    )
                    if pos is not None:
                        self.joint_pos[i] = pos
                        self.joint_vel[i] = vel
                        self.joint_tau[i] = tau
                        self.joint_tem[i] = tem
                except Exception:
                    pass

    def start_transition_to_sit(self, duration: float = 3.0):
        """Starts smooth S-curve transition from current pose to current tuned sitting target (0.0 + delta_q)."""
        with self.lock:
            for i in range(P.N_JOINTS):
                if self.motors[i] is not None:
                    try:
                        self.motors[i].enable_motor()
                        self.motors[i].set_run_mode("CONTROL_MODE")
                    except Exception:
                        pass

            self.start_pos = self.joint_pos.copy()
            self.transition_start_time = time.time()
            self.transition_duration = duration
            self.transition_progress = 0.0
            self.state = "TRANSITIONING"
            self.is_active = True
            self.status_msg = f"⏳ Memulai transisi mulus ke posisi DUDUK ({duration:.1f}s S-curve)..."

    def _control_loop(self):
        loop_dt = self.control_dt
        while self.running:
            t_start = time.time()

            try:
                with self.lock:
                    active = self.is_active
                    current_state = self.state
                    kp_val = self.kp
                    kd_val = self.kd
                    target_q = self.delta_q.copy()
                    dur = self.transition_duration

                if active:
                    if current_state == "TRANSITIONING":
                        elapsed = time.time() - self.transition_start_time
                        progress = min(1.0, elapsed / max(0.1, dur))

                        # Smooth S-Curve (Cosine) Trajectory
                        s = 0.5 * (1.0 - math.cos(math.pi * progress))
                        desired_pos = self.start_pos + s * (target_q - self.start_pos)

                        with self.lock:
                            self.cmd_pos = desired_pos.copy()
                            self.transition_progress = progress

                        if progress >= 1.0:
                            with self.lock:
                                self.state = "HOLDING"
                                self.status_msg = f"✅ Posisi duduk tercapai & HOLD. Tekan [↑] atau [↓] untuk setel {self.selected_name}."
                    elif current_state == "HOLDING":
                        # Smoothly follow delta_q with rate limit
                        with self.lock:
                            for i in range(P.N_JOINTS):
                                diff = target_q[i] - self.cmd_pos[i]
                                max_step = self.max_rate * loop_dt
                                if abs(diff) > max_step:
                                    self.cmd_pos[i] += math.copysign(max_step, diff)
                                else:
                                    self.cmd_pos[i] = target_q[i]

                # Send CAN commands & read feedback
                for i in range(P.N_JOINTS):
                    motor = self.motors[i]
                    if motor is not None:
                        try:
                            if not active or current_state == "PASSIVE":
                                _, pos, vel, tau, tem = motor.send_control_command(
                                    p_ref=0.0, v_ref=0.0, kp=0.0, kd=0.0, tau_ff=0.0
                                )
                            else:
                                is_coxa = (i % 3 == 0)
                                kp_cmd = self.kp_coxa if is_coxa else kp_val
                                kd_cmd = self.kd_coxa if is_coxa else kd_val
                                _, pos, vel, tau, tem = motor.send_control_command(
                                    p_ref=self.cmd_pos[i], v_ref=0.0, kp=kp_cmd, kd=kd_cmd, tau_ff=0.0
                                )

                            if pos is not None:
                                with self.lock:
                                    self.joint_pos[i] = pos
                                    self.joint_vel[i] = vel
                                    self.joint_tau[i] = tau
                                    self.joint_tem[i] = tem

                                # Over-torque safety failsafe (>14.0 Nm)
                                if abs(tau) > 14.0 and active:
                                    self.is_active = False
                                    self.state = "PASSIVE"
                                    self.status_msg = f"🚨 [FAILSAFE] Over-torque pada {P.JOINT_NAME[i]} ({abs(tau):.1f} Nm > 14.0 Nm). Motor dinonaktifkan!"
                        except Exception:
                            pass
            except Exception:
                pass

            elapsed = time.time() - t_start
            sleep_time = max(0.001, loop_dt - elapsed)
            time.sleep(sleep_time)

    def adjust_selected(self, delta: float):
        """Adjusts target angle for currently selected joint(s) and directly moves motor."""
        with self.lock:
            indices = self.selected_indices
            name = self.selected_name
            for idx in indices:
                self.delta_q[idx] += delta

            sample_val = self.delta_q[indices[0]]
            sign_str = "+" if delta > 0 else ""

            if not self.is_active or self.state == "PASSIVE":
                self.start_transition_to_sit(duration=2.5)
            else:
                self.status_msg = f"⚙️ {name}: {sign_str}{delta:+.2f} rad (Δq: {sample_val:+.3f} rad | Offset Baru: {self.base_offsets[indices[0]] - sample_val:+.4f})"

    def select_joint_by_key(self, key: str):
        """Selects joint by letter/number key according to active mode."""
        k = key.lower()
        mapping = PAIRED_SELECTION_MAP if self.mode == "PAIRED" else SINGLE_SELECTION_MAP
        if k in mapping:
            with self.lock:
                self.active_key = k
                self._update_selection()
                self.status_msg = f"👉 Dipilih: {C_BOLD}{self.selected_name}{C_RESET} (Tekan [↑] atau [↓] untuk gerakkan motor)"

    def _on_gamepad_edge(self, state: XboxState, edges: Dict[str, Any]):
        """Gamepad button event handler."""
        with self.lock:
            if "btn_a" in edges:
                self.start_transition_to_sit(duration=3.0)
            elif "btn_y" in edges:
                self.toggle_mode()
            elif "btn_lb" in edges or "btn_back" in edges or "btn_x" in edges:
                self.is_active = False
                self.state = "PASSIVE"
                self.status_msg = "🛑 [Xbox Stop] Motor PASIF (Zero Torque)."
            elif "dpad_up" in edges:
                self.adjust_selected(+self.step_size)
            elif "dpad_down" in edges:
                self.adjust_selected(-self.step_size)

    def handle_key(self, char: str):
        """Processes terminal single keypress / Arrow keys."""
        with self.lock:
            self.last_key_pressed = f"'{char}'"

        # 1. UP Arrow / (+) / (=) -> Calibrate Selected Joint UP
        if char in ["KEY_UP", '+', '=']:
            self.adjust_selected(+self.step_size)
            return

        # 2. DOWN Arrow / (-) / (_) -> Calibrate Selected Joint DOWN
        elif char in ["KEY_DOWN", '-', '_']:
            self.adjust_selected(-self.step_size)
            return

        # 3. Enter / Return -> Engage / Smooth Sit & Hold
        elif char == "ENTER":
            self.start_transition_to_sit(duration=3.0)
            return

        # 4. Mode Toggle (M / TAB)
        elif char in ['m', 'M', "TAB"]:
            self.toggle_mode()
            return

        # 5. Precision Step Toggle (P)
        elif char in ['p', 'P']:
            with self.lock:
                self.step_size = 0.01 if self.step_size >= 0.04 else 0.05
                self.status_msg = f"🎯 Resolusi Step diubah ke: ±{self.step_size:.2f} rad ({self.step_size*180/math.pi:.1f}°)"
            return

        # 6. Space / Passive
        elif char in [' ', "SPACE"]:
            with self.lock:
                self.is_active = False
                self.state = "PASSIVE"
                self.status_msg = "🛑 [SPACE] Motor PASIF (Zero Torque). Torsi dimatikan."
            return

        # 7. Reset
        elif char == '0':
            with self.lock:
                for idx in self.selected_indices:
                    self.delta_q[idx] = 0.0
                self.status_msg = f"🔄 Δq pada {self.selected_name} di-reset ke 0.00 rad."
            return

        # 8. Save & Exit
        elif char in ['s', 'S']:
            self.save_requested = True
            self.running = False
            return

        # 9. Ctrl+C
        elif char == "CTRL_C":
            self.running = False
            return

        # 10. Joint Selection Keys
        c = char.lower()
        self.select_joint_by_key(c)

    def render_dashboard(self):
        """Renders clear, intuitive UI dashboard."""
        with self.lock:
            active = self.is_active
            st = self.state
            prog = self.transition_progress
            pos = self.joint_pos.copy()
            tau = self.joint_tau.copy()
            deltas = self.delta_q.copy()
            step = self.step_size
            status = self.status_msg
            mode = self.mode
            sel_name = self.selected_name
            sel_indices = self.selected_indices
            last_k = self.last_key_pressed

        new_offsets = self.base_offsets - deltas

        if st == "TRANSITIONING":
            state_str = f"{C_MAGENTA}⏳ TRANSISI DUDUK ({prog*100:.0f}%){C_RESET}"
        elif st == "HOLDING" and active:
            state_str = f"{C_GREEN}● AKTIF HOLD (SITTING){C_RESET}"
        else:
            state_str = f"{C_YELLOW}○ PASIF (ZERO TORQUE){C_RESET}"

        mode_badge = f"{C_BOLD}{C_CYAN}[PAIRED (Kiri+Kanan Simetris)]{C_RESET}" if mode == "PAIRED" else f"{C_BOLD}{C_GREEN}[SINGLE (Kiri/Kanan Terpisah)]{C_RESET}"

        out = []
        out.append(C_CLEAR)
        out.append(f"{C_BOLD}{C_CYAN}╔══════════════════════════════════════════════════════════════════════════════════════════════╗{C_RESET}")
        out.append(f"{C_BOLD}{C_CYAN}║             🐾 NXP JAGUAR: TRUE ZERO (SIT) INTERACTIVE CALIBRATION TOOL                      ║{C_RESET}")
        out.append(f"{C_BOLD}{C_CYAN}╚══════════════════════════════════════════════════════════════════════════════════════════════╝{C_RESET}")
        out.append(f" Status Motor : {state_str}   |   Mode: {mode_badge} (Tekan {C_BOLD}[M]{C_RESET} untuk Toggle)")
        out.append(f" Joint Dipilih: {C_BOLD}{C_YELLOW}👉 {sel_name}{C_RESET}  ===>  {C_BOLD}{C_GREEN}Tekan [↑] atau [↓] untuk gerakkan motor (±{step:.2f} rad){C_RESET}")
        out.append(f" Pesan Sistem : {C_WHITE}{status}{C_RESET}  [Key Input: {C_BOLD}{C_MAGENTA}{last_k}{C_RESET}]")
        out.append("─" * 94)

        # Quick Key Selector Grid based on Mode
        out.append(f"{C_BOLD}📌 PILIHAN JOINT ({mode} MODE):{C_RESET}")
        if mode == "PAIRED":
            out.append(f"   [Kaki Depan]   : {C_BOLD}[1 / Q / Z]{C_RESET} Kedua Coxa Depan | {C_BOLD}[2 / W / X]{C_RESET} Kedua Hip Depan | {C_BOLD}[3 / E / C]{C_RESET} Kedua Knee Depan")
            out.append(f"   [Kaki Belakang]: {C_BOLD}[4 / R / V]{C_RESET} Kedua Coxa Blkg  | {C_BOLD}[5 / T / B]{C_RESET} Kedua Hip Blkg  | {C_BOLD}[6 / Y / N]{C_RESET} Kedua Knee Blkg")
        else:
            out.append(f"   [FL Depan Kiri]   : {C_BOLD}[1/Q]{C_RESET} Coxa   | {C_BOLD}[2/W]{C_RESET} Hip   | {C_BOLD}[3/E]{C_RESET} Knee")
            out.append(f"   [FR Depan Kanan]  : {C_BOLD}[4/R]{C_RESET} Coxa   | {C_BOLD}[5/T]{C_RESET} Hip   | {C_BOLD}[6/Y]{C_RESET} Knee")
            out.append(f"   [BL Belakang Kiri]: {C_BOLD}[7/A]{C_RESET} Coxa   | {C_BOLD}[8/D]{C_RESET} Hip   | {C_BOLD}[9/F]{C_RESET} Knee")
            out.append(f"   [BR Belakang Kanan: {C_BOLD}[U/G]{C_RESET} Coxa   | {C_BOLD}[I/H]{C_RESET} Hip   | {C_BOLD}[O/J]{C_RESET} Knee")
        out.append("─" * 94)

        # Table Header
        out.append(f"{C_BOLD}{'Sel':<4} {'Pilih':<7} {'Leg':<4} {'Joint Name':<18} {'Bus/ID':<8} {'Offset Lama':<13} {'Δq Tune':<12} {'Pos Ukur':<11} {'Torsi':<9} {'Offset Baru':<12}{C_RESET}")
        out.append("─" * 94)

        idx_to_single_key = {6: '1/Q', 7: '2/W', 8: '3/E', 9: '4/R', 10: '5/T', 11: '6/Y', 0: '7/A', 1: '8/D', 2: '9/F', 3: 'U/G', 4: 'I/H', 5: 'O/J'}
        idx_to_paired_key = {6: '1/Z', 7: '2/X', 8: '3/C', 9: '1/Z', 10: '2/X', 11: '3/C', 0: '4/V', 1: '5/B', 2: '6/N', 3: '4/V', 4: '5/B', 5: '6/N'}

        display_order = [6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5]
        leg_names = {6: 'FL', 7: 'FL', 8: 'FL', 9: 'FR', 10: 'FR', 11: 'FR', 0: 'BL', 1: 'BL', 2: 'BL', 3: 'BR', 4: 'BR', 5: 'BR'}

        for i in display_order:
            k = idx_to_paired_key.get(i, ' ') if mode == "PAIRED" else idx_to_single_key.get(i, ' ')
            leg = leg_names[i]
            jname = P.JOINT_NAME[i]
            bus_id = f"{P.DEVICE[i]} #{P.CAN_ID[i]}"
            old_off = self.base_offsets[i]
            dq = deltas[i]
            cur_p = pos[i]
            t = tau[i]
            new_off = new_offsets[i]

            is_selected = (i in sel_indices)
            sel_mark = f"{C_BOLD}{C_GREEN}👉{C_RESET}" if is_selected else "  "
            key_str = f"{C_BG_BLUE}[{k}]{C_RESET}" if is_selected else f"{C_BOLD}{C_YELLOW}[{k}]{C_RESET}"
            name_str = f"{C_BOLD}{C_YELLOW}{jname:<18}{C_RESET}" if is_selected else f"{jname:<18}"

            dq_str = f"{C_GREEN}{dq:+7.3f} rad{C_RESET}" if abs(dq) > 1e-4 else f"{dq:+7.3f} rad"
            new_off_str = f"{C_BOLD}{C_CYAN}{new_off:+8.4f}{C_RESET}" if abs(dq) > 1e-4 else f"{new_off:+8.4f}"
            tau_str = f"{C_RED}{t:+5.1f} Nm{C_RESET}" if abs(t) > 6.0 else f"{t:+5.1f} Nm"

            out.append(f"{sel_mark}  {key_str:<15} {leg:<4} {name_str} {bus_id:<8} {old_off:+8.4f} rad  {dq_str:<21} {cur_p:+7.3f} rad {tau_str:<18} {new_off_str}")

            if i in [8, 11, 2]:
                out.append("╌" * 94)

        out.append("─" * 94)
        out.append(f"{C_BOLD}🎮 PANDUAN PENGGUNAAN:{C_RESET}")
        out.append(f"   • {C_BOLD}[M]{C_RESET} : {C_BOLD}{C_CYAN}TOGGLE MODE (SINGLE ↔ PAIRED){C_RESET}")
        out.append(f"   • {C_BOLD}{C_GREEN}[↑] / [↓] atau [+] / [-]{C_RESET} : {C_BOLD}{C_GREEN}GERAKKAN MOTOR JOINT TERPILIH (±{step:.2f} rad){C_RESET}")
        out.append(f"   • {C_BOLD}[Enter]{C_RESET} : Transisi Duduk & Hold | {C_BOLD}[P]{C_RESET} : Ganti Step (±0.05 / ±0.01) | {C_BOLD}[SPACE]{C_RESET} : Torsi OFF")
        out.append(f"   • {C_BOLD}{C_GREEN}[S]{C_RESET} : {C_BOLD}{C_GREEN}SIMPAN & KELUAR (Cetak Offset Baru){C_RESET}")
        out.append(f"{C_BOLD}{C_CYAN}══════════════════════════════════════════════════════════════════════════════════════════════╝{C_RESET}")

        # In raw mode, lines need \r\n
        sys.stdout.write("\r\n".join(out) + "\r\n")
        sys.stdout.flush()

    def print_final_offsets(self):
        """Prints copy-pasteable calibration offsets for parameters.py and C++ header."""
        new_offsets = self.base_offsets - self.delta_q

        print("\r\n" + "=" * 80)
        print(f"{C_BOLD}{C_GREEN}🎉 KALIBRASI POSISI SITTING SELESAI! HASIL OFFSET SUDUT BARU:{C_RESET}")
        print("=" * 80)

        print("\r\n" + f"{C_BOLD}1. Tabel Perbandingan Offset (Python Joint Frame):{C_RESET}")
        print(f"{'Joint Name':<20} {'Offset Lama':<16} {'Δq Penyesuaian':<18} {'Offset Baru (rad)':<18} {'Offset Baru (deg)':<18}")
        print("-" * 88)
        for i in range(P.N_JOINTS):
            old_off = self.base_offsets[i]
            dq = self.delta_q[i]
            new_off = new_offsets[i]
            deg = new_off * 180.0 / math.pi
            print(f"{P.JOINT_NAME[i]:<20} {old_off:+10.4f} rad     {dq:+10.4f} rad       {C_BOLD}{new_off:+10.4f} rad{C_RESET}       {deg:+8.2f}°")
        print("-" * 88)

        print("\r\n" + f"{C_BOLD}2. Salin ke {C_CYAN}scripts/parameters.py{C_RESET} :{C_RESET}")
        print("```python")
        print("MOTOR_OFFSET_ANGLE = [")
        print(f"    {new_offsets[0]:+7.4f}, {new_offsets[1]:+7.4f}, {new_offsets[2]:+7.4f},  # BL (can1: Collar, Hip, Knee)")
        print(f"    {new_offsets[3]:+7.4f}, {new_offsets[4]:+7.4f}, {new_offsets[5]:+7.4f},  # BR (can0: Collar, Hip, Knee)")
        print(f"    {new_offsets[6]:+7.4f}, {new_offsets[7]:+7.4f}, {new_offsets[8]:+7.4f},  # FL (can1: Collar, Hip, Knee)")
        print(f"    {new_offsets[9]:+7.4f}, {new_offsets[10]:+7.4f}, {new_offsets[11]:+7.4f},  # FR (can0: Collar, Hip, Knee)")
        print("]")
        print("```")

        # C++ uses opposite sign
        cpp_offsets = -new_offsets
        print("\r\n" + f"{C_BOLD}3. Salin ke {C_CYAN}include/jaguar_control/robstride_hardware_manager.hpp{C_RESET} :{C_RESET}")
        print("```cpp")
        print(f'joint_configs_[0] = {{"BL_collar_joint", "can1", 4,  1, {cpp_offsets[0]:+7.4f}, -0.40,  0.40, 20.0, 17.0, {{}}}};')
        print(f'joint_configs_[1] = {{"BL_hip_joint",    "can1", 5, -1, {cpp_offsets[1]:+7.4f}, -3.14,  3.14, 20.0, 17.0, {{}}}};')
        print(f'joint_configs_[2] = {{"BL_knee_joint",   "can1", 6, -1, {cpp_offsets[2]:+7.4f}, -0.10,  2.80, 20.0, 17.0, {{}}}};')
        print(f'joint_configs_[3] = {{"BR_collar_joint", "can0", 4,  1, {cpp_offsets[3]:+7.4f}, -0.40,  0.40, 20.0, 17.0, {{}}}};')
        print(f'joint_configs_[4] = {{"BR_hip_joint",    "can0", 5,  1, {cpp_offsets[4]:+7.4f}, -3.14,  3.14, 20.0, 17.0, {{}}}};')
        print(f'joint_configs_[5] = {{"BR_knee_joint",   "can0", 6,  1, {cpp_offsets[5]:+7.4f}, -0.10,  2.80, 20.0, 17.0, {{}}}};')
        print(f'joint_configs_[6] = {{"FL_collar_joint", "can1", 1, -1, {cpp_offsets[6]:+7.4f}, -0.40,  0.40, 20.0, 17.0, {{}}}};')
        print(f'joint_configs_[7] = {{"FL_hip_joint",    "can1", 2, -1, {cpp_offsets[7]:+7.4f}, -3.14,  3.14, 20.0, 17.0, {{}}}};')
        print(f'joint_configs_[8] = {{"FL_knee_joint",   "can1", 3, -1, {cpp_offsets[8]:+7.4f}, -0.10,  2.80, 20.0, 17.0, {{}}}};')
        print(f'joint_configs_[9] = {{"FR_collar_joint", "can0", 1, -1, {cpp_offsets[9]:+7.4f}, -0.40,  0.40, 20.0, 17.0, {{}}}};')
        print(f'joint_configs_[10] ={{"FR_hip_joint",    "can0", 2,  1, {cpp_offsets[10]:+7.4f}, -3.14,  3.14, 20.0, 17.0, {{}}}};')
        print(f'joint_configs_[11] ={{"FR_knee_joint",   "can0", 3,  1, {cpp_offsets[11]:+7.4f}, -0.10,  2.80, 20.0, 17.0, {{}}}};')
        print("```")
        print("=" * 80 + "\r\n")

    def shutdown(self):
        """Safely de-energizes all motors and closes CAN sockets."""
        self.running = False
        self.is_active = False
        print(f"\r\n{C_YELLOW}Mematikan seluruh torsi motor secara aman...{C_RESET}\r\n")
        for i in range(P.N_JOINTS):
            motor = self.motors[i]
            if motor is not None:
                try:
                    motor.send_control_command(p_ref=0.0, v_ref=0.0, kp=0.0, kd=0.0, tau_ff=0.0)
                    motor.disable_motor()
                except Exception:
                    pass
        print(f"{C_GREEN}✅ Seluruh motor telah dinonaktifkan (Torque Disabled).{C_RESET}\r\n")


def main():
    calibrator = SitZeroCalibrator()
    key_reader = TerminalInputHandler(callback=calibrator.handle_key)
    key_reader.start()

    try:
        while calibrator.running:
            calibrator.render_dashboard()
            time.sleep(0.1)  # 10 Hz dashboard refresh
    except KeyboardInterrupt:
        pass
    finally:
        key_reader.stop()
        calibrator.shutdown()
        if calibrator.save_requested:
            calibrator.print_final_offsets()


if __name__ == "__main__":
    main()
