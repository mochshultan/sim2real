#!/usr/bin/env python3
"""
NXP Jaguar Sim-to-Sim Interactive Simulator (MuJoCo).
Loads latest Isaac Lab DreamWaQ JIT policy and runs real-time 50 Hz control with keyboard / gamepad teleop.
"""

import os
import sys
import glob
import time
import select
import tty
import termios
import threading
import struct
import fcntl
import argparse
import numpy as np
import torch
import mujoco
import mujoco.viewer

from observation_builder import (
    ObservationBuilder,
    MUJOCO_TO_ISAAC,
    ISAAC_TO_MUJOCO,
    DEFAULT_JOINT_POS_ISAAC,
    DEFAULT_JOINT_POS_MUJOCO,
    DEFAULT_JOINT_POS_DICT,
    DEFAULT_BASE_HEIGHT,
    ROBOT_CONFIG_PATH,
    quat_rotate_inverse,
)

# ==============================================================================
# 1. AUTO-FIND / CUSTOM-FLAG JIT POLICY LOADER (DREAMWAQ CENET)
# ==============================================================================
def export_checkpoint_to_jit(model_path: str, export_path: str) -> bool:
    """Exports a raw DreamWaQ model_*.pt checkpoint to TorchScript JIT policy.pt."""
    try:
        from isaaclab_tasks.manager_based.locomotion.velocity.config.nxp_jaguar.dreamwaq import (
            DreamWaQActorCritic,
            FusedDreamWaQPolicy,
        )
        ac = DreamWaQActorCritic(
            history_len=5,
            obs_dim=45,
            critic_dim=48,
            action_dim=12,
            latent_dim=16,
            vel_dim=3,
        ).to("cpu")
        ckpt = torch.load(model_path, map_location="cpu")
        ac.load_state_dict(ckpt["model_state_dict"])
        ac.eval()

        fused = FusedDreamWaQPolicy(ac.cenet_encoder, ac.actor, history_len=5, obs_dim=45).to("cpu")
        dummy_in = torch.zeros(1, 5, 45, device="cpu")
        traced = torch.jit.trace(fused, dummy_in)
        os.makedirs(os.path.dirname(os.path.abspath(export_path)), exist_ok=True)
        traced.save(export_path)
        print(f"[INFO] Auto-exported DreamWaQ checkpoint {os.path.basename(model_path)} -> {export_path}")
        return True
    except Exception as e:
        print(f"[WARNING] Could not auto-export checkpoint {model_path}: {e}")
        return False


def find_latest_policy(requested_path=None, load_run=None, task=None) -> str:
    """Finds or auto-exports the newest DreamWaQ policy JIT file."""
    if requested_path:
        req_p = os.path.expanduser(requested_path)
        if os.path.isfile(req_p):
            if req_p.endswith(".pt") and "model_" in os.path.basename(req_p):
                export_p = os.path.join(os.path.dirname(req_p), "exported", "policy.pt")
                export_checkpoint_to_jit(req_p, export_p)
                return export_p
            return os.path.abspath(req_p)
        elif os.path.isdir(req_p):
            for cand in [os.path.join(req_p, "policy.pt"), os.path.join(req_p, "exported", "policy.pt")]:
                if os.path.isfile(cand):
                    return os.path.abspath(cand)
            # Check if model_*.pt exists in folder and auto-export
            model_files = [f for f in os.listdir(req_p) if f.startswith("model_") and f.endswith(".pt")]
            if model_files:
                model_files.sort(key=lambda x: int(x.split("_")[1].split(".")[0]) if x.split("_")[1].split(".")[0].isdigit() else 0)
                latest_m = os.path.join(req_p, model_files[-1])
                export_p = os.path.join(req_p, "exported", "policy.pt")
                export_checkpoint_to_jit(latest_m, export_p)
                return os.path.abspath(export_p)
            recursive_pt = glob.glob(os.path.join(req_p, "**", "*.pt"), recursive=True)
            if recursive_pt:
                recursive_pt.sort(key=os.path.getmtime, reverse=True)
                return os.path.abspath(recursive_pt[0])
            raise FileNotFoundError(f"No .pt file found in custom directory: {req_p}")

    search_patterns = [
        os.path.expanduser("~/IsaacLab/logs/dreamwaq/*/*/policy.pt"),
        os.path.expanduser("~/IsaacLab/logs/dreamwaq/*/*/policy_*.pt"),
        os.path.expanduser("~/jaguar_sim2real/models/policy.pt"),
        os.path.expanduser("~/jaguar_sim2sim/models/policy.pt"),
    ]

    all_policies = []
    for pattern in search_patterns:
        for m in glob.glob(pattern):
            all_policies.append((os.path.getmtime(m), m))

    if not all_policies:
        raise FileNotFoundError("No exported DreamWaQ policy.pt found in IsaacLab logs or models folder!")

    all_policies.sort(key=lambda x: x[0], reverse=True)
    latest_policy = all_policies[0][1]
    print(f"\n[INFO] Loaded latest DreamWaQ JIT policy:\n       -> {latest_policy}\n")
    return os.path.abspath(latest_policy)

# ==============================================================================
# 2. KEYBOARD & LINUX JOYSTICK (XBOX CONTROLLER) TELEOP HANDLER
# ==============================================================================
class LinuxJoystickHandler:
    """
    Reads Linux joystick events directly from /dev/input/js* via native struct unpacking.
    Supports Microsoft Xbox 360 / One / Series S|X controllers with full axis, button,
    trigger modifiers, deadzones, and automatic reconnection.
    """
    # Linux joystick event structure: time (u32), value (s16), type (u8), number (u8)
    EVENT_FORMAT = "=IhBB"
    EVENT_SIZE = struct.calcsize(EVENT_FORMAT)
    JS_EVENT_BUTTON = 0x01
    JS_EVENT_AXIS = 0x02
    JS_EVENT_INIT = 0x80

    def __init__(self, dev_path: str = "/dev/input/js0", deadzone: float = 0.08):
        self.dev_path = dev_path
        self.deadzone = deadzone
        self.axes = [0.0] * 16
        self.buttons = [0] * 32
        self.prev_buttons = [0] * 32
        self.prev_dpad_x = 0
        self.prev_dpad_y = 0
        self.connected = False
        self.device_name = "Not Detected"
        self.running = False
        self.thread = None
        self.lock = threading.Lock()
        self.stick_was_active = False

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._worker, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=0.5)

    def _get_device_name(self, fd: int) -> str:
        try:
            buf = bytearray(64)
            # JSIOCGNAME(len) = 0x80006a13 + (len << 16)
            fcntl.ioctl(fd, 0x80006a13 + (0x10000 * len(buf)), buf)
            name = buf.split(b'\x00')[0].decode('utf-8', 'ignore').strip()
            return name if name else "Xbox / Linux Gamepad"
        except Exception:
            return "Xbox / Linux Gamepad"

    def _worker(self):
        while self.running:
            if not os.path.exists(self.dev_path):
                with self.lock:
                    self.connected = False
                    self.device_name = f"Device {self.dev_path} not found"
                time.sleep(1.0)
                continue

            try:
                fd = os.open(self.dev_path, os.O_RDONLY | os.O_NONBLOCK)
                dev_name = self._get_device_name(fd)
                with self.lock:
                    self.connected = True
                    self.device_name = dev_name
                print(f"\n[INFO] Gamepad connected: {dev_name} ({self.dev_path})")

                while self.running:
                    rlist, _, _ = select.select([fd], [], [], 0.05)
                    if not self.running:
                        break
                    if rlist:
                        while True:
                            try:
                                data = os.read(fd, self.EVENT_SIZE)
                                if len(data) == self.EVENT_SIZE:
                                    _, val, ev_type, num = struct.unpack(self.EVENT_FORMAT, data)
                                    actual_type = ev_type & ~self.JS_EVENT_INIT
                                    with self.lock:
                                        if actual_type == self.JS_EVENT_BUTTON:
                                            if num < len(self.buttons):
                                                self.buttons[num] = val
                                        elif actual_type == self.JS_EVENT_AXIS:
                                            if num < len(self.axes):
                                                self.axes[num] = float(val) / 32767.0
                                else:
                                    break
                            except (BlockingIOError, InterruptedError):
                                break
                            except Exception:
                                break
                os.close(fd)
            except Exception:
                with self.lock:
                    self.connected = False
                time.sleep(1.0)

    def _apply_deadzone(self, val: float) -> float:
        if abs(val) < self.deadzone:
            return 0.0
        sign = 1.0 if val > 0.0 else -1.0
        return sign * (abs(val) - self.deadzone) / (1.0 - self.deadzone)

    def process_input(self, teleop):
        if not self.connected:
            return

        with self.lock:
            curr_buttons = list(self.buttons)
            curr_axes = list(self.axes)

        # ----------------------------------------------------------------------
        # 1. Edge-Triggered Buttons (Xbox Controller on /dev/input/js*)
        # ----------------------------------------------------------------------
        # Button 0: A -> STANDUP (Berdiri Halus Bebas Oleng)
        if curr_buttons[0] and not self.prev_buttons[0]:
            teleop.set_state("STANDUP", "Xbox [A] -> STANDUP (Berdiri Tegak)")
            teleop.last_input_source = "gamepad"

        # Button 1: B -> STANDBY (Duduk di Lantai Posisi 0)
        if curr_buttons[1] and not self.prev_buttons[1]:
            teleop.set_state("STANDBY", "Xbox [B] -> STANDBY (Duduk di Lantai)")
            teleop.last_input_source = "gamepad"

        # Button 2: X -> WALK (RL DreamWaQ Policy Aktif)
        if curr_buttons[2] and not self.prev_buttons[2]:
            teleop.set_state("WALK", "Xbox [X] -> WALK (RL Policy Aktif)")
            teleop.last_input_source = "gamepad"

        # Button 3: Y -> STOP (Zero Velocity)
        if curr_buttons[3] and not self.prev_buttons[3]:
            teleop.stop("Xbox [Y] -> STOP (Zero Velocity)")
            teleop.last_input_source = "gamepad"

        # Button 6: Back / View -> Reset Simulation
        if curr_buttons[6] and not self.prev_buttons[6]:
            teleop.request_reset("Xbox [Back] -> Reset Robot Simulation")
            teleop.last_input_source = "gamepad"

        # Button 7: Start / Menu -> Cycle State
        if curr_buttons[7] and not self.prev_buttons[7]:
            if teleop.state == "STANDBY":
                teleop.set_state("STANDUP", "Xbox [Start] -> STANDUP")
            elif teleop.state == "STANDUP":
                teleop.set_state("WALK", "Xbox [Start] -> WALK")
            else:
                teleop.set_state("STANDUP", "Xbox [Start] -> STANDUP")
            teleop.last_input_source = "gamepad"

        # Button 9 (L3) or 10 (R3): Stick Click Stop
        if (curr_buttons[9] and not self.prev_buttons[9]) or (curr_buttons[10] and not self.prev_buttons[10]):
            teleop.stop("Xbox [Stick Click] -> STOP")
            teleop.last_input_source = "gamepad"

        # ----------------------------------------------------------------------
        # 2. D-pad (Axes 6 & 7) Edge-Triggered
        # ----------------------------------------------------------------------
        dpad_x = 1 if curr_axes[6] > 0.5 else (-1 if curr_axes[6] < -0.5 else 0)
        dpad_y = 1 if curr_axes[7] > 0.5 else (-1 if curr_axes[7] < -0.5 else 0)

        if dpad_y == -1 and self.prev_dpad_y != -1:  # D-pad Up
            teleop.set_state("WALK", "Xbox [D-Pad Up] -> WALK")
            teleop.last_input_source = "gamepad"
        elif dpad_y == 1 and self.prev_dpad_y != 1:  # D-pad Down
            teleop.set_state("STANDBY", "Xbox [D-Pad Down] -> STANDBY")
            teleop.last_input_source = "gamepad"
        elif dpad_x != 0 and self.prev_dpad_x == 0:  # D-pad Left/Right
            teleop.set_state("STANDUP", "Xbox [D-Pad] -> STANDUP")
            teleop.last_input_source = "gamepad"

        self.prev_buttons = curr_buttons
        self.prev_dpad_x = dpad_x
        self.prev_dpad_y = dpad_y

        # ----------------------------------------------------------------------
        # 3. Continuous Analog Stick Movement
        # ----------------------------------------------------------------------
        # Axis 0: Left Stick X  -> Lateral vy (Geser Kiri/Kanan)
        # Axis 1: Left Stick Y  -> Linear vx  (Maju/Mundur; negative=up)
        # Axis 3: Right Stick X -> Yaw wz     (Putar Kiri/Kanan; negative=left)
        lx = self._apply_deadzone(curr_axes[0])
        ly = self._apply_deadzone(curr_axes[1])
        rx = self._apply_deadzone(curr_axes[3])

        # Speed Modifiers:
        # RB (button 5) or RT (axis 5 > 0) -> Turbo Mode (1.35x)
        # LB (button 4) or LT (axis 2 > 0) -> Precision Slow Mode (0.50x)
        speed_mult = 1.0
        mode_tag = ""
        if curr_buttons[5] or (len(curr_axes) > 5 and curr_axes[5] > 0.0):
            speed_mult = 1.35
            mode_tag = " [TURBO]"
        elif curr_buttons[4] or (len(curr_axes) > 2 and curr_axes[2] > 0.0):
            speed_mult = 0.50
            mode_tag = " [SLOW]"

        is_stick_active = (abs(lx) > 1e-4 or abs(ly) > 1e-4 or abs(rx) > 1e-4)

        if is_stick_active:
            # Vx: Axis 1 (Negative is Up/Forward, Positive is Down/Backward)
            if ly < 0.0:
                vx = (-ly) * 1.2 * speed_mult
            else:
                vx = (-ly) * 0.8 * speed_mult

            # Vy: Axis 0 (Negative is Left -> +Vy, Positive is Right -> -Vy)
            vy = (-lx) * 0.6 * speed_mult

            # Wz: Axis 3 (Negative is Left -> +Wz CCW, Positive is Right -> -Wz CW)
            wz = (-rx) * 1.5 * speed_mult

            teleop.cmd_vel[0] = vx
            teleop.cmd_vel[1] = vy
            teleop.cmd_vel[2] = wz
            self.stick_was_active = True
            teleop.last_input_source = "gamepad"
            if teleop.state == "WALK":
                teleop.last_action_msg = f"Xbox: Vx={vx:+.2f} Vy={vy:+.2f} Wz={wz:+.2f}{mode_tag}"
        else:
            if self.stick_was_active:
                teleop.cmd_vel[:] = 0.0
                self.stick_was_active = False
                if teleop.state == "WALK":
                    teleop.last_action_msg = "Xbox: Stick Netral (0.0 m/s)"


class TerminalInputHandler:
    """Reads non-blocking single-character inputs directly from terminal stdin."""
    def __init__(self, teleop):
        self.teleop = teleop
        self.running = True
        self.thread = threading.Thread(target=self._worker, daemon=True)
        self.old_settings = None

    def start(self):
        try:
            if sys.stdin.isatty():
                self.old_settings = termios.tcgetattr(sys.stdin)
                tty.setcbreak(sys.stdin.fileno())
            self.thread.start()
        except Exception as e:
            print(f"[WARN] Terminal cbreak mode not enabled: {e}")

    def _worker(self):
        while self.running:
            try:
                if sys.stdin.isatty():
                    rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
                    if rlist:
                        char = sys.stdin.read(1)
                        if char:
                            self.teleop.handle_char(char)
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


class TeleopController:
    def __init__(self, joystick_dev: str = "/dev/input/js0", enable_joystick: bool = True):
        self.cmd_vel = np.array([0.0, 0.0, 0.0], dtype=np.float32)  # [vx, vy, wz]
        self.state = "STANDBY"  # States: STANDBY -> STANDUP -> WALK
        self.state_changed = False
        self.reset_requested = False
        self.last_action_msg = "Standby (Posisi 0 di Lantai)"
        self.last_input_source = "keyboard"
        self.joystick_dev = joystick_dev
        self.enable_joystick = enable_joystick
        self.joystick = None

        if self.enable_joystick:
            self.joystick = LinuxJoystickHandler(dev_path=joystick_dev)
            self.joystick.start()

    def set_state(self, new_state: str, action_msg: str = ""):
        if self.state != new_state:
            self.state = new_state
            self.state_changed = True
            if new_state in ["STANDBY", "STANDUP"]:
                self.cmd_vel[:] = 0.0
            if action_msg:
                self.last_action_msg = action_msg

    def stop(self, action_msg: str = "Berhenti (Stop)"):
        self.cmd_vel[:] = 0.0
        self.last_action_msg = action_msg

    def request_reset(self, action_msg: str = "Reset Robot Simulation (Posisi 0)"):
        self.reset_requested = True
        self.cmd_vel[:] = 0.0
        self.last_action_msg = action_msg

    def handle_char(self, char: str):
        """Processes a single keystroke from terminal or GUI."""
        c = char.upper()
        # W / S : Forward / Backward
        if c == 'W':
            self.cmd_vel[0] = min(1.2, self.cmd_vel[0] + 0.2)
            self.last_action_msg = "KB: Maju (Vx +0.2)"
            self.last_input_source = "keyboard"
        elif c == 'S':
            self.cmd_vel[0] = max(-0.8, self.cmd_vel[0] - 0.2)
            self.last_action_msg = "KB: Mundur (Vx -0.2)"
            self.last_input_source = "keyboard"
        # A / D : Lateral Left / Right
        elif c == 'A':
            self.cmd_vel[1] = min(0.6, self.cmd_vel[1] + 0.15)
            self.last_action_msg = "KB: Geser Kiri (Vy +0.15)"
            self.last_input_source = "keyboard"
        elif c == 'D':
            self.cmd_vel[1] = max(-0.6, self.cmd_vel[1] - 0.15)
            self.last_action_msg = "KB: Geser Kanan (Vy -0.15)"
            self.last_input_source = "keyboard"
        # Q / E : Turn Left / Right
        elif c == 'Q':
            self.cmd_vel[2] = min(1.5, self.cmd_vel[2] + 0.3)
            self.last_action_msg = "KB: Putar Kiri (Wz +0.3)"
            self.last_input_source = "keyboard"
        elif c == 'E':
            self.cmd_vel[2] = max(-1.5, self.cmd_vel[2] - 0.3)
            self.last_action_msg = "KB: Putar Kanan (Wz -0.3)"
            self.last_input_source = "keyboard"
        # Space : Stop
        elif char == ' ':
            self.stop("KB: Berhenti (Stop)")
            self.last_input_source = "keyboard"
        # 1 : STANDBY
        elif char == '1':
            self.set_state("STANDBY", "KB: -> STANDBY (Duduk di Posisi 0)")
            self.last_input_source = "keyboard"
        # 2 : STANDUP
        elif char == '2':
            self.set_state("STANDUP", "KB: -> STANDUP (Berdiri Tegak)")
            self.last_input_source = "keyboard"
        # 3 : WALK
        elif char == '3':
            self.set_state("WALK", "KB: -> WALK (RL Policy Aktif)")
            self.last_input_source = "keyboard"
        # R : Reset
        elif c == 'R':
            self.request_reset("KB: Reset Simulation (Posisi 0)")
            self.last_input_source = "keyboard"

    def on_key(self, keycode):
        """Callback from MuJoCo viewer window."""
        try:
            self.handle_char(chr(keycode))
        except Exception:
            pass

    def update_gamepad(self):
        if self.joystick is not None and self.joystick.connected:
            self.joystick.process_input(self)


def quat_to_euler_deg(q):
    """Converts quaternion [w, x, y, z] to Euler angles in degrees (roll, pitch, yaw)."""
    w, x, y, z = q[0], q[1], q[2], q[3]
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = np.arcsin(np.clip(sinp, -1.0, 1.0))

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return np.rad2deg(roll), np.rad2deg(pitch), np.rad2deg(yaw)


def get_stance_base_height(mj_model, mj_data) -> tuple[float, list[str]]:
    """
    Measures vertical clearance from robot base to stance feet (Stance Foot Kinematics).
    Immune to terrain raycast drop errors on stairs, slopes, and rough terrain.
    Returns:
        (stance_base_height, list_of_contacting_feet)
    """
    foot_names = ["Fr_tibia_link", "Fl_tibia_link", "Br_tibia_link", "Bl_tibia_link"]
    foot_labels = {"Fr_tibia_link": "FR", "Fl_tibia_link": "FL", "Br_tibia_link": "BR", "Bl_tibia_link": "BL"}

    body_names = [mj_model.body(i).name for i in range(mj_model.nbody)]
    foot_body_ids = {name: mj_model.body(name).id for name in foot_names if name in body_names}

    base_z = float(mj_data.qpos[2])
    stance_heights = []
    stance_feet = set()

    for i in range(mj_data.ncon):
        c = mj_data.contact[i]
        b1 = mj_model.geom_bodyid[c.geom1]
        b2 = mj_model.geom_bodyid[c.geom2]
        for name, b_id in foot_body_ids.items():
            if b_id in (b1, b2):
                h = base_z - float(c.pos[2])
                stance_heights.append(h)
                stance_feet.add(foot_labels[name])

    if stance_heights:
        return float(np.mean(stance_heights)), sorted(list(stance_feet))

    # Fallback when all feet in air: height relative to lowest tibia frame
    if foot_body_ids:
        min_foot_z = min([mj_data.xpos[b_id][2] for b_id in foot_body_ids.values()])
        return float(base_z - min_foot_z), ["AIR"]

    return base_z, ["N/A"]


def get_base_height_relative(mj_model, mj_data) -> float:
    """Measures vertical clearance from robot base to ground using raycasting (like IsaacLab RayCaster)."""
    pnt = mj_data.qpos[0:3].copy()
    vec = np.array([0.0, 0.0, -1.0], dtype=np.float64)
    geomid = np.zeros(1, dtype=np.int32)
    body_id = mj_model.body("base_link").id if "base_link" in [mj_model.body(i).name for i in range(mj_model.nbody)] else -1
    dist = mujoco.mj_ray(mj_model, mj_data, pnt, vec, None, 1, body_id, geomid)
    if dist >= 0:
        return float(dist)
    return float(mj_data.qpos[2])


def print_terminal_dashboard(teleop: TeleopController, terrain: str, mj_model, mj_data, target_pos_mujoco: np.ndarray):
    """Renders a clean, live, in-place terminal dashboard table showing robot states & joint readings."""
    vx, vy, wz = teleop.cmd_vel
    z_abs = mj_data.qpos[2]
    z_stance, stance_feet = get_stance_base_height(mj_model, mj_data)
    stance_str = ",".join(stance_feet) if stance_feet else "None"
    z_rel = get_base_height_relative(mj_model, mj_data)
    roll, pitch, yaw = quat_to_euler_deg(mj_data.qpos[3:7])

    state_color = {
        "STANDBY": "\033[1;33m",  # Yellow
        "STANDUP": "\033[1;36m",  # Cyan
        "WALK":    "\033[1;32m",  # Green
    }.get(teleop.state, "\033[1;37m")
    reset_c = "\033[0m"
    cyan_c = "\033[1;36m"
    bold_c = "\033[1m"
    dim_c = "\033[2m"
    green_c = "\033[1;32m"
    yellow_c = "\033[1;33m"

    if teleop.joystick and teleop.joystick.connected:
        joy_tag = f"{green_c}🎮 {teleop.joystick.device_name[:26]} ({teleop.joystick.dev_path}){reset_c}"
    else:
        joy_tag = f"{yellow_c}❌ Disconnected ({teleop.joystick_dev}){reset_c}"

    legs = [
        ("FR (Depan Kn) ", 0, 1, 2),
        ("FL (Depan Kr) ", 3, 4, 5),
        ("BR (Blkg Kn)  ", 6, 7, 8),
        ("BL (Blkg Kr)  ", 9, 10, 11),
    ]

    qpos_m = mj_data.qpos[7:19]
    torques_m = mj_data.ctrl[:]

    lines = []
    lines.append("\033[H")  # Move cursor to top-left (no clear screen = flicker-free)
    lines.append("┌────────────────────────────────────────────────────────────────────────────────────────┐")
    lines.append(f"│ 🐾 {bold_c}NXP JAGUAR SIM-TO-SIM DASHBOARD{reset_c}  | State: {state_color}{teleop.state:<8}{reset_c} | Terrain: {cyan_c}{terrain.upper():<9}{reset_c}│")
    lines.append(f"│ Cmd : Vx={vx:+5.2f} m/s | Vy={vy:+5.2f} m/s | Wz={wz:+5.2f} rad/s | Action: {teleop.last_action_msg:<26}│")
    lines.append(f"│ IMU : Stance-H={z_stance:5.2f}m [{stance_str:<11}] | Ray-Z={z_rel:5.2f}m | Abs-Z={z_abs:5.2f}m | R/P/Y=[{roll:+4.0f}°,{pitch:+4.0f}°,{yaw:+4.0f}°]│")
    lines.append(f"│ Joy : {joy_tag:<82}│")
    lines.append("├────────────────────────────────────────────────────────────────────────────────────────┤")
    lines.append("│ Kaki          │ Joint     │ Measured [rad] │ Target [rad]   │ Error [rad]   │ Torque [Nm]   │")
    lines.append("├───────────────┼───────────┼────────────────┼────────────────┼───────────────┼───────────────┤")

    for leg_name, r_idx, h_idx, k_idx in legs:
        for j_name, idx in [("Roll", r_idx), ("Hip ", h_idx), ("Knee", k_idx)]:
            q_cur = qpos_m[idx]
            q_tgt = target_pos_mujoco[idx]
            q_err = q_tgt - q_cur
            tau = torques_m[idx]
            deg = np.rad2deg(q_cur)

            leg_str = leg_name if j_name == "Roll" else "               "
            lines.append(f"│ {leg_str} │ {j_name}      │ {q_cur:+6.3f} ({deg:+5.1f}°) │ {q_tgt:+6.3f}         │ {q_err:+6.3f}        │ {tau:+6.2f}        │")
        lines.append("├───────────────┼───────────┼────────────────┼────────────────┼───────────────┼───────────────┤")

    lines[-1] = "└───────────────┴───────────┴────────────────┴────────────────┴───────────────┴───────────────┘"
    lines.append(f" {dim_c}[KB: W/S/A/D/Q/E/Space | Xbox: L-Stick=Maju/Geser, R-Stick=Putar, A=Stand, B=Sit, X=Walk, Y=Stop]{reset_c}\033[K")

    sys.stdout.write("\n".join(lines) + "\n")
    sys.stdout.flush()

# ==============================================================================
# 3. PROCEDURAL TERRAIN GENERATOR FOR MUJOCO
# ==============================================================================
def generate_terrain(mj_model, terrain_type="flat"):
    """Generates procedural terrain heightfields in MuJoCo."""
    if terrain_type == "flat" or mj_model.nhfield == 0:
        return

    nrow = mj_model.hfield_nrow[0]
    ncol = mj_model.hfield_ncol[0]
    x = np.linspace(-6, 6, ncol)
    y = np.linspace(-6, 6, nrow)
    X, Y = np.meshgrid(x, y)

    if terrain_type == "rough":
        # Multi-scale undulating natural bumps & rolling mounds (height ~ 2-7 cm)
        Z = (0.35 * np.sin(1.2 * X) * np.cos(1.2 * Y) +
             0.25 * np.sin(2.5 * X + 0.8 * Y) +
             0.20 * np.cos(3.2 * Y - 0.5 * X) +
             0.15 * np.random.uniform(0, 0.3, (nrow, ncol)))
        # Flat spawn circle at center (radius < 0.7m)
        dist = np.sqrt(X**2 + Y**2)
        blend = np.clip((dist - 0.7) / 0.6, 0.0, 1.0)
        Z = Z * blend
    elif terrain_type == "stairs":
        # Concentric stepped pyramid stairs (3.5 cm step height)
        dist_sq = np.maximum(np.abs(X), np.abs(Y))
        steps = np.floor(dist_sq / 0.6)
        Z = np.clip(steps * 0.035, 0.0, 0.20)
        Z[dist_sq < 0.8] = 0.0
    elif terrain_type == "obstacles":
        # Discrete stepping stone blocks
        Z = np.zeros((nrow, ncol), dtype=np.float32)
        rng = np.random.RandomState(42)
        for _ in range(35):
            ox = rng.uniform(-4.5, 4.5)
            oy = rng.uniform(-4.5, 4.5)
            if np.sqrt(ox**2 + oy**2) < 1.0:
                continue
            w = rng.uniform(0.3, 0.7)
            h = rng.uniform(0.02, 0.06)
            mask = (np.abs(X - ox) < w) & (np.abs(Y - oy) < w)
            Z[mask] = np.maximum(Z[mask], h)
    else:
        return

    Z_norm = (Z - Z.min()) / (Z.max() - Z.min() + 1e-6)
    mj_model.hfield_data[:] = Z_norm.flatten()
    print(f"[INFO] Procedural terrain '{terrain_type.upper()}' generated ({nrow}x{ncol} grid).")


# ==============================================================================
# 4. MAIN SIM-TO-SIM SIMULATION LOOP
# ==============================================================================
def run_sim2sim(policy_path=None, load_run=None, task=None, terrain="flat", joystick_dev="/dev/input/js0", enable_joystick=True):
    # Select scene XML based on terrain choice
    if terrain in ["rough", "stairs", "obstacles"]:
        model_xml = os.path.join(os.path.dirname(__file__), "models/scene_rough.xml")
    else:
        model_xml = os.path.join(os.path.dirname(__file__), "models/scene.xml")

    if not os.path.isfile(model_xml):
        raise FileNotFoundError(f"Model XML not found at {model_xml}")

    # Load MuJoCo Model & Populate Terrain
    mj_model = mujoco.MjModel.from_xml_path(model_xml)
    generate_terrain(mj_model, terrain)
    mj_data = mujoco.MjData(mj_model)

    # Load Policy (Auto or Custom Flag)
    policy_file = find_latest_policy(policy_path, load_run, task)
    policy = torch.jit.load(policy_file, map_location="cpu")
    policy.eval()

    obs_builder = ObservationBuilder()
    teleop = TeleopController(joystick_dev=joystick_dev, enable_joystick=enable_joystick)

    # Start non-blocking terminal input listener
    term_input = TerminalInputHandler(teleop)
    term_input.start()

    # Robot Parameters (RobStride RS00)
    ACTION_SCALE = 0.25
    TORQUE_LIMIT = 17.0
    CONTROL_DT = 0.02   # 50 Hz control loop
    SIM_DT = mj_model.opt.timestep  # 0.002s (500 Hz)
    DECIMATION = int(CONTROL_DT / SIM_DT)

    # Joint Angle References (Dynamically loaded directly from nxp_jaguar.py)
    q_sit_isaac = np.zeros(12, dtype=np.float32)            # All 12 joints at 0.0 rad (Calibrated Position 0)
    q_stand_isaac = DEFAULT_JOINT_POS_ISAAC.copy()          # Nominal standing q0 from nxp_jaguar.py

    # Smooth Minimum-Jerk Trajectory Interpolation State
    transition_start_time = time.time()
    transition_duration = 2.0
    transition_start_q = q_sit_isaac.copy()
    transition_goal_q = q_sit_isaac.copy()
    in_transition = False

    # Initial Pose Setup (Start resting on ground at Position 0: 0.0 rad)
    def reset_robot():
        nonlocal in_transition, transition_start_time, transition_start_q, transition_goal_q
        mujoco.mj_resetData(mj_model, mj_data)
        mj_data.qpos[0:3] = [0.0, 0.0, 0.12]  # resting on ground
        mj_data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]  # quat [w, x, y, z]
        mj_data.qpos[7:19] = 0.0  # all joints at 0.0 (Calibrated Position 0)
        mj_data.qvel[:] = 0.0
        mujoco.mj_forward(mj_model, mj_data)
        teleop.reset_requested = False
        teleop.state = "STANDBY"
        teleop.state_changed = False
        teleop.cmd_vel[:] = 0.0
        in_transition = False
        transition_goal_q = q_sit_isaac.copy()

    reset_robot()

    target_pos_isaac = q_sit_isaac.copy()
    target_pos_mujoco = q_sit_isaac[ISAAC_TO_MUJOCO].copy()

    print("=" * 75)
    print(" 🐾 NXP JAGUAR: INTERACTIVE MUJOCO SIM-TO-SIM SIMULATOR")
    print(f" 🏔️  MEDAN SIMULASI : [{terrain.upper()}]")
    print(f" 📌 STAND POSE SRC  : [{ROBOT_CONFIG_PATH}]")
    print(f" 📐 STAND HEIGHT    : {DEFAULT_BASE_HEIGHT:.2f} m")
    print("=" * 75)
    print(" 🕹️  KEYBOARD CONTROLS (Langsung ketik di Terminal atau Window):")
    print("    [1]         : Set State to STANDBY (Duduk di Posisi 0)")
    print("    [2]         : Set State to STANDUP (Berdiri Halus Bebas Oleng)")
    print("    [3]         : Set State to WALK    (Aktifkan RL DreamWaQ Policy)")
    print("    [W / S]     : Maju / Mundur        (Linear Velocity X)")
    print("    [A / D]     : Geser Kiri / Kanan   (Linear Velocity Y)")
    print("    [Q / E]     : Putar Kiri / Kanan   (Angular Velocity Yaw)")
    print("    [Space]     : Berhenti             (Zero Velocity)")
    print("    [R]         : Reset Robot ke Posisi 0")
    print(" 🎮 XBOX CONTROLLER CONTROLS (/dev/input/js0):")
    print("    [Left Stick]       : Maju/Mundur (Vx) & Geser Kiri/Kanan (Vy)")
    print("    [Right Stick]      : Putar Kiri/Kanan (Yaw Rate Wz)")
    print("    [Button A]         : STANDUP (Berdiri Halus Bebas Oleng)")
    print("    [Button B]         : STANDBY (Duduk di Posisi 0)")
    print("    [Button X]         : WALK    (Aktifkan RL DreamWaQ Policy)")
    print("    [Button Y]         : Stop    (Zero Velocity)")
    print("    [Button Back/View] : Reset Robot Simulation ke Posisi 0")
    print("    [Button Start/Menu]: Cycle State (STANDBY -> STANDUP -> WALK)")
    print("    [Button RB / RT]   : Turbo Speed Boost (1.35x)")
    print("    [Button LB / LT]   : Precision Slow Mode (0.50x)")
    print("    [D-Pad Up/Down]    : Mode WALK / Mode STANDBY")
    print("=" * 75)
    print(" [INFO] Robot mulai di POSISI 0 (Joint = 0.0 rad di lantai).")
    print(" [TIPS] Tekan [2] / Xbox [A] untuk Berdiri, lalu [3] / Xbox [X] untuk Jalan!\n")

    # Clear terminal screen for clean live dashboard display
    sys.stdout.write("\033[2J\033[H")
    sys.stdout.flush()

    step_counter = 0

    try:
        with mujoco.viewer.launch_passive(mj_model, mj_data, key_callback=teleop.on_key) as viewer:
            # Initial Camera View Setup (3rd-person isometric follow)
            viewer.cam.distance = 1.6
            viewer.cam.elevation = -20.0
            viewer.cam.azimuth = 115.0

            while viewer.is_running():
                step_start = time.time()

                if teleop.reset_requested:
                    reset_robot()

                teleop.update_gamepad()

                # 50 Hz Control Decimation
                if step_counter % DECIMATION == 0:
                    # 1. Read Sensors from MuJoCo
                    base_quat = mj_data.qpos[3:7].copy()  # [w, x, y, z]
                    v_world = mj_data.qvel[0:3].copy()    # world frame velocity
                    base_lin_vel = quat_rotate_inverse(base_quat, v_world)  # body frame velocity
                    base_ang_vel = mj_data.sensordata[0:3].copy()           # gyro in body frame
                    
                    # Joint positions & velocities in MuJoCo order
                    curr_joint_pos_mujoco = mj_data.qpos[7:19].copy()
                    curr_joint_vel_mujoco = mj_data.qvel[6:18].copy()

                    # Remap to Isaac Lab order
                    curr_joint_pos_isaac = curr_joint_pos_mujoco[MUJOCO_TO_ISAAC]
                    curr_joint_vel_isaac = curr_joint_vel_mujoco[MUJOCO_TO_ISAAC]

                    # Check State Transition Triggers
                    if teleop.state_changed:
                        teleop.state_changed = False
                        if teleop.state == "STANDUP":
                            in_transition = True
                            transition_start_time = time.time()
                            transition_start_q = curr_joint_pos_isaac.copy()
                            transition_goal_q = q_stand_isaac.copy()
                        elif teleop.state == "STANDBY":
                            in_transition = True
                            transition_start_time = time.time()
                            transition_start_q = curr_joint_pos_isaac.copy()
                            transition_goal_q = q_sit_isaac.copy()
                        elif teleop.state == "WALK":
                            in_transition = False
                            init_obs = obs_builder.build_step_observation(
                                base_ang_vel,
                                base_quat,
                                teleop.cmd_vel,
                                curr_joint_pos_isaac,
                                curr_joint_vel_isaac,
                            )
                            obs_builder.reset_history(init_obs)

                    # 2. State Machine Trajectory Generation (Minimum Jerk)
                    if in_transition:
                        elapsed = time.time() - transition_start_time
                        alpha = min(1.0, elapsed / transition_duration)
                        # Minimum-Jerk polynomial: zero velocity and acceleration at boundary
                        s = 10.0 * (alpha ** 3) - 15.0 * (alpha ** 4) + 6.0 * (alpha ** 5)
                        target_pos_isaac = (1.0 - s) * transition_start_q + s * transition_goal_q
                        if alpha >= 1.0:
                            in_transition = False
                    elif teleop.state == "STANDBY":
                        target_pos_isaac = q_sit_isaac.copy()
                    elif teleop.state == "STANDUP":
                        target_pos_isaac = q_stand_isaac.copy()
                    elif teleop.state == "WALK":
                        # Build 45-D Step Observation & Update 5-Step History Buffer (1, 5, 45)
                        obs_45d = obs_builder.build_step_observation(
                            base_ang_vel,
                            base_quat,
                            teleop.cmd_vel,
                            curr_joint_pos_isaac,
                            curr_joint_vel_isaac,
                        )
                        history_tensor = obs_builder.update_and_get_history(obs_45d)

                        # RL Model Forward Pass (TorchScript JIT - FusedDreamWaQPolicy)
                        with torch.no_grad():
                            actions = policy(history_tensor)

                        raw_action = actions.squeeze(0).cpu().numpy()
                        obs_builder.update_last_action(raw_action)
                        target_pos_isaac = q_stand_isaac + ACTION_SCALE * raw_action

                    # Convert target positions back to MuJoCo order
                    target_pos_mujoco = target_pos_isaac[ISAAC_TO_MUJOCO]

                # 3. PD Motor Control (500 Hz Physics Step)
                # Use high-stability gains during standup/stop, training gains during active walk
                if teleop.state == "WALK":
                    is_zero_cmd = np.linalg.norm(teleop.cmd_vel) < 1e-3
                    if is_zero_cmd:
                        kp_now, kd_now = 32.0, 1.8
                    else:
                        # Match sim2real gains: Coxa Kp=18, Kd=1.0 | Hip/Knee Kp=25, Kd=1.5
                        kp_now = np.array([18.0, 25.0, 25.0, 18.0, 25.0, 25.0, 18.0, 25.0, 25.0, 18.0, 25.0, 25.0], dtype=np.float32)
                        kd_now = np.array([1.0, 1.5, 1.5, 1.0, 1.5, 1.5, 1.0, 1.5, 1.5, 1.0, 1.5, 1.5], dtype=np.float32)
                elif in_transition or teleop.state == "STANDUP":
                    kp_now, kd_now = 35.0, 2.0
                else:
                    kp_now, kd_now = 15.0, 1.0

                curr_pos_m = mj_data.qpos[7:19]
                curr_vel_m = mj_data.qvel[6:18]
                torques = kp_now * (target_pos_mujoco - curr_pos_m) - kd_now * curr_vel_m
                torques = np.clip(torques, -TORQUE_LIMIT, TORQUE_LIMIT)
                mj_data.ctrl[:] = torques

                # Step Physics
                mujoco.mj_step(mj_model, mj_data)
                step_counter += 1

                # 4. Render Live In-Place Terminal Dashboard Table (10 Hz)
                if step_counter % (DECIMATION * 5) == 0:
                    print_terminal_dashboard(teleop, terrain, mj_model, mj_data, target_pos_mujoco)

                # 5. Sync Viewer & Smooth Camera Auto Follow (at ~60 Hz)
                if step_counter % (DECIMATION * 2) == 0:
                    # Camera smoothly tracks robot root position
                    cam_alpha = 0.15
                    viewer.cam.lookat[0] += cam_alpha * (mj_data.qpos[0] - viewer.cam.lookat[0])
                    viewer.cam.lookat[1] += cam_alpha * (mj_data.qpos[1] - viewer.cam.lookat[1])
                    viewer.cam.lookat[2] += cam_alpha * ((mj_data.qpos[2] + 0.05) - viewer.cam.lookat[2])
                    viewer.sync()

                # Real-time synchronization
                elapsed = time.time() - step_start
                if SIM_DT > elapsed:
                    time.sleep(SIM_DT - elapsed)
    finally:
        term_input.stop()
        if teleop.joystick is not None:
            teleop.joystick.stop()
        print("\n[INFO] Sim-to-sim simulation ended.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="NXP Jaguar Sim-to-Sim MuJoCo Runner")
    parser.add_argument(
        "--policy",
        type=str,
        default=None,
        help="Path to specific TorchScript policy.pt or directory containing policy.pt (e.g. /path/to/exported/)"
    )
    parser.add_argument(
        "--load_run",
        type=str,
        default=None,
        help="Specific training run name/timestamp to load (e.g. 2026-08-14_19-14-32)"
    )
    parser.add_argument(
        "--task",
        type=str,
        default=None,
        choices=["rough", "rough_trot", "flat"],
        help="Task environment to prioritize when auto-resolving latest checkpoint ('rough', 'rough_trot', or 'flat')"
    )
    parser.add_argument(
        "--terrain",
        type=str,
        default="flat",
        choices=["flat", "rough", "stairs", "obstacles"],
        help="Terrain type in MuJoCo: 'flat', 'rough' (rolling hills), 'stairs' (stepped pyramid), or 'obstacles' (stepping stones)"
    )
    parser.add_argument(
        "--joystick",
        type=str,
        default="/dev/input/js0",
        help="Linux joystick device path for Xbox controller (default: /dev/input/js0)"
    )
    parser.add_argument(
        "--no_joystick",
        action="store_true",
        help="Disable joystick / gamepad teleoperation"
    )
    args = parser.parse_args()

    run_sim2sim(
        policy_path=args.policy,
        load_run=args.load_run,
        task=args.task,
        terrain=args.terrain,
        joystick_dev=args.joystick,
        enable_joystick=not args.no_joystick,
    )
