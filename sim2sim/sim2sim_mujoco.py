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
    quat_rotate_inverse,
)

# ==============================================================================
# 1. AUTO-FIND / CUSTOM-FLAG JIT POLICY LOADER
# ==============================================================================
def find_latest_policy(requested_path=None, load_run=None, task=None) -> str:
    """
    Finds the TorchScript JIT policy:
    1. If requested_path is given:
       - If it is a file (e.g. policy.pt), use it directly.
       - If it is a directory (e.g. .../2026-08-14_19-14-32/exported/), auto-find policy.pt inside it.
    2. If load_run is given (e.g. 2026-08-14_19-14-32):
       - Search for that specific run in IsaacLab logs.
    3. If task is given ('rough' or 'flat'):
       - Find the latest run for that specific task.
    4. Default:
       - Auto-find the newest policy.pt across all runs and tasks.
    """
    if requested_path:
        req_p = os.path.expanduser(requested_path)
        if os.path.isfile(req_p):
            print(f"\n[INFO] Loaded custom JIT policy file:\n       -> {os.path.abspath(req_p)}\n")
            return os.path.abspath(req_p)
        elif os.path.isdir(req_p):
            candidates = [
                os.path.join(req_p, "policy.pt"),
                os.path.join(req_p, "exported", "policy.pt"),
            ]
            for c in candidates:
                if os.path.isfile(c):
                    print(f"\n[INFO] Loaded JIT policy from custom directory:\n       -> {os.path.abspath(c)}\n")
                    return os.path.abspath(c)
            # Search recursively in the directory
            recursive_pt = glob.glob(os.path.join(req_p, "**", "*.pt"), recursive=True)
            if recursive_pt:
                recursive_pt.sort(key=os.path.getmtime, reverse=True)
                print(f"\n[INFO] Auto-found JIT policy in directory:\n       -> {os.path.abspath(recursive_pt[0])}\n")
                return os.path.abspath(recursive_pt[0])
            raise FileNotFoundError(f"No .pt file found in custom directory: {req_p}")

    if load_run:
        pattern = os.path.expanduser(f"~/IsaacLab/logs/rsl_rl/*/{load_run}/**/policy.pt")
        matches = glob.glob(pattern, recursive=True)
        if matches:
            matches.sort(key=os.path.getmtime, reverse=True)
            print(f"\n[INFO] Loaded JIT policy from run '{load_run}':\n       -> {os.path.abspath(matches[0])}\n")
            return os.path.abspath(matches[0])
        raise FileNotFoundError(f"Could not find exported policy.pt for run: {load_run}")

    # Build search patterns based on task
    if task:
        task_folders = [f"nxp_jaguar_{task}"]
    else:
        task_folders = ["nxp_jaguar_rough", "nxp_jaguar_flat"]

    search_patterns = []
    for tf in task_folders:
        search_patterns.append(os.path.expanduser(f"~/IsaacLab/logs/rsl_rl/{tf}/*/exported/policy.pt"))
    search_patterns.append(os.path.expanduser("~/mevius2_ws_ros-o/src/mevius2-master/models/policy.pt"))

    all_policies = []
    for pattern in search_patterns:
        for m in glob.glob(pattern):
            all_policies.append((os.path.getmtime(m), m))

    if not all_policies:
        raise FileNotFoundError("No exported policy.pt found in IsaacLab logs or models folder!")

    all_policies.sort(key=lambda x: x[0], reverse=True)
    latest_policy = all_policies[0][1]
    print(f"\n[INFO] Auto-resolved newest trained JIT policy:")
    print(f"       -> {latest_policy}\n")
    return os.path.abspath(latest_policy)

# ==============================================================================
# 2. KEYBOARD & TERMINAL TELEOP HANDLER
# ==============================================================================
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
    def __init__(self):
        self.cmd_vel = np.array([0.0, 0.0, 0.0], dtype=np.float32)  # [vx, vy, wz]
        self.state = "STANDBY"  # States: STANDBY -> STANDUP -> WALK
        self.state_changed = False
        self.reset_requested = False

        # Try initialize joystick if available
        self.has_joystick = False
        try:
            import pygame
            pygame.init()
            pygame.joystick.init()
            if pygame.joystick.get_count() > 0:
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
                self.has_joystick = True
                print(f"[INFO] Gamepad detected: {self.joystick.get_name()}")
        except Exception:
            pass

    def handle_char(self, char: str):
        """Processes a single keystroke from terminal or GUI."""
        c = char.upper()
        # W / S : Forward / Backward
        if c == 'W':
            self.cmd_vel[0] = min(1.2, self.cmd_vel[0] + 0.2)
            self._print_status("Maju (Vx +0.2)")
        elif c == 'S':
            self.cmd_vel[0] = max(-0.8, self.cmd_vel[0] - 0.2)
            self._print_status("Mundur (Vx -0.2)")
        # A / D : Lateral Left / Right
        elif c == 'A':
            self.cmd_vel[1] = min(0.6, self.cmd_vel[1] + 0.15)
            self._print_status("Geser Kiri (Vy +0.15)")
        elif c == 'D':
            self.cmd_vel[1] = max(-0.6, self.cmd_vel[1] - 0.15)
            self._print_status("Geser Kanan (Vy -0.15)")
        # Q / E : Turn Left / Right
        elif c == 'Q':
            self.cmd_vel[2] = min(1.5, self.cmd_vel[2] + 0.3)
            self._print_status("Putar Kiri (Wz +0.3)")
        elif c == 'E':
            self.cmd_vel[2] = max(-1.5, self.cmd_vel[2] - 0.3)
            self._print_status("Putar Kanan (Wz -0.3)")
        # Space : Stop
        elif char == ' ':
            self.cmd_vel[:] = 0.0
            self._print_status("Berhenti (Stop)")
        # 1 : STANDBY
        elif char == '1':
            if self.state != "STANDBY":
                self.state = "STANDBY"
                self.state_changed = True
                self.cmd_vel[:] = 0.0
                self._print_status("Transisi -> STANDBY (Duduk ke Posisi 0)")
        # 2 : STANDUP
        elif char == '2':
            if self.state != "STANDUP":
                self.state = "STANDUP"
                self.state_changed = True
                self.cmd_vel[:] = 0.0
                self._print_status("Transisi -> STANDUP (Berdiri halus ke q0)")
        # 3 : WALK
        elif char == '3':
            if self.state != "WALK":
                self.state = "WALK"
                self.state_changed = True
                self._print_status("State -> WALK (RL DreamWaQ Policy Active)")
        # R : Reset
        elif c == 'R':
            self.reset_requested = True
            self.cmd_vel[:] = 0.0
            self._print_status("Reset Robot Simulation (Posisi 0)")

    def on_key(self, keycode):
        """Callback from MuJoCo viewer window."""
        try:
            self.handle_char(chr(keycode))
        except Exception:
            pass

    def _print_status(self, action_str=""):
        vx, vy, wz = self.cmd_vel
        sys.stdout.write(f"\r\033[K [{self.state:<7}] Cmd: [vx={vx:+5.2f} m/s, vy={vy:+5.2f} m/s, wz={wz:+5.2f} rad/s] | {action_str}\n")
        sys.stdout.flush()

    def update_gamepad(self):
        if not self.has_joystick:
            return
        import pygame
        pygame.event.pump()
        # Buttons
        if self.joystick.get_button(0):  # X / A
            if self.state != "STANDUP":
                self.state = "STANDUP"
                self.state_changed = True
        elif self.joystick.get_button(1):  # O / B
            if self.state != "WALK":
                self.state = "WALK"
                self.state_changed = True
        elif self.joystick.get_button(2):  # Square / X
            if self.state != "STANDBY":
                self.state = "STANDBY"
                self.state_changed = True
                self.cmd_vel[:] = 0.0

        # Axes
        self.cmd_vel[0] = -self.joystick.get_axis(1) * 1.0  # Left stick Y -> vx
        self.cmd_vel[1] = -self.joystick.get_axis(0) * 0.5  # Left stick X -> vy
        self.cmd_vel[2] = -self.joystick.get_axis(3) * 1.2  # Right stick X -> wz

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
def run_sim2sim(policy_path=None, load_run=None, task=None, terrain="flat"):
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
    teleop = TeleopController()

    # Start non-blocking terminal input listener
    term_input = TerminalInputHandler(teleop)
    term_input.start()

    # Robot Parameters (RobStride RS00)
    ACTION_SCALE = 0.25
    TORQUE_LIMIT = 17.0
    CONTROL_DT = 0.02   # 50 Hz control loop
    SIM_DT = mj_model.opt.timestep  # 0.002s (500 Hz)
    DECIMATION = int(CONTROL_DT / SIM_DT)

    # Joint Angle References
    q_sit_isaac = np.zeros(12, dtype=np.float32)            # All 12 joints at 0.0 rad (Position 0)
    q_stand_isaac = DEFAULT_JOINT_POS_ISAAC.copy()          # Nominal standing q0 = [-1.5, 1.5]

    # Smooth Minimum-Jerk Trajectory Interpolation State
    transition_start_time = time.time()
    transition_duration = 2.0
    transition_start_q = q_sit_isaac.copy()
    transition_goal_q = q_sit_isaac.copy()
    in_transition = False

    # Initial Pose Setup (Start resting on ground at Position 0)
    def reset_robot():
        nonlocal in_transition, transition_start_time, transition_start_q, transition_goal_q
        mujoco.mj_resetData(mj_model, mj_data)
        mj_data.qpos[0:3] = [0.0, 0.0, 0.12]  # resting on ground
        mj_data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]  # quat [w, x, y, z]
        mj_data.qpos[7:19] = 0.0  # all joints at 0.0 (Position 0)
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
    print("=" * 75)
    print(" [INFO] Robot mulai di POSISI 0 (Joint = 0.0 rad di lantai).")
    print(" [TIPS] Tekan [2] untuk Berdiri Halus (Standup), lalu [3] untuk Jalan!\n")

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
                        # Build 48-D Observation Tensor
                        obs = obs_builder.build_observation(
                            base_lin_vel,
                            base_ang_vel,
                            base_quat,
                            teleop.cmd_vel,
                            curr_joint_pos_isaac,
                            curr_joint_vel_isaac,
                        )

                        # RL Model Forward Pass (TorchScript JIT)
                        with torch.no_grad():
                            actions = policy(obs)
                            obs_builder.update_last_action(actions)

                        raw_action = actions.squeeze(0).numpy()
                        is_zero_cmd = np.linalg.norm(teleop.cmd_vel) < 1e-3
                        if is_zero_cmd:
                            # Settle smoothly and cleanly to nominal standing pose q0
                            target_pos_isaac = q_stand_isaac + (ACTION_SCALE * raw_action) * 0.15
                        else:
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
                        kp_now, kd_now = 25.0, 1.0
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

                # Sync Viewer & Smooth Camera Auto Follow (at ~60 Hz)
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
        choices=["rough", "flat"],
        help="Task environment to prioritize when auto-resolving latest checkpoint ('rough' or 'flat')"
    )
    parser.add_argument(
        "--terrain",
        type=str,
        default="flat",
        choices=["flat", "rough", "stairs", "obstacles"],
        help="Terrain type in MuJoCo: 'flat', 'rough' (rolling hills), 'stairs' (stepped pyramid), or 'obstacles' (stepping stones)"
    )
    args = parser.parse_args()

    run_sim2sim(policy_path=args.policy, load_run=args.load_run, task=args.task, terrain=args.terrain)
