#!/usr/bin/env python3
"""
NXP Jaguar Isaac Lab Sim-to-Sim Interactive Runner with Viser Visualizer & Terminal Teleop.
Runs 1 single robot on rough or flat terrain with full keyboard teleoperation and 3D WebGL Viser viewer.
"""

import os
import sys
import glob
import time
import argparse
import select
import tty
import termios
import threading
import numpy as np

# ==============================================================================
# 1. AUTO-FIND LATEST JIT / CHECKPOINT POLICY
# ==============================================================================
def find_latest_policy(requested_path=None, load_run=None, task=None) -> str:
    """Finds the TorchScript JIT policy in logs directory."""
    if requested_path:
        req_p = os.path.expanduser(requested_path)
        if os.path.isfile(req_p):
            return os.path.abspath(req_p)
        elif os.path.isdir(req_p):
            for c in [os.path.join(req_p, "policy.pt"), os.path.join(req_p, "exported", "policy.pt")]:
                if os.path.isfile(c):
                    return os.path.abspath(c)
            recursive_pt = glob.glob(os.path.join(req_p, "**", "*.pt"), recursive=True)
            if recursive_pt:
                recursive_pt.sort(key=os.path.getmtime, reverse=True)
                return os.path.abspath(recursive_pt[0])

    if load_run:
        pattern = os.path.expanduser(f"~/IsaacLab/logs/rsl_rl/*/{load_run}/**/policy.pt")
        matches = glob.glob(pattern, recursive=True)
        if matches:
            matches.sort(key=os.path.getmtime, reverse=True)
            return os.path.abspath(matches[0])

    task_folders = [f"nxp_jaguar_{task}"] if task else ["nxp_jaguar_rough", "nxp_jaguar_flat"]
    all_policies = []
    for tf in task_folders:
        for m in glob.glob(os.path.expanduser(f"~/IsaacLab/logs/rsl_rl/{tf}/*/exported/policy.pt")):
            all_policies.append((os.path.getmtime(m), m))

    if not all_policies:
        return os.path.expanduser("~/mevius2_ws_ros-o/src/mevius2-master/models/policy.pt")

    all_policies.sort(key=lambda x: x[0], reverse=True)
    return os.path.abspath(all_policies[0][1])


# ==============================================================================
# 2. TERMINAL TELEOP INPUT HANDLER
# ==============================================================================
class TerminalInputHandler:
    """Non-blocking keyboard reader directly from terminal stdin."""
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
            print(f"[WARN] Terminal input mode not enabled: {e}")

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
        self.state = "STANDUP"  # Default start standing
        self.reset_requested = False

    def handle_char(self, char: str):
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
            self.state = "STANDBY"
            self.cmd_vel[:] = 0.0
            self._print_status("State -> STANDBY")
        # 2 : STANDUP
        elif char == '2':
            self.state = "STANDUP"
            self.cmd_vel[:] = 0.0
            self._print_status("State -> STANDUP")
        # 3 : WALK
        elif char == '3':
            self.state = "WALK"
            self._print_status("State -> WALK (RL DreamWaQ Policy Active)")
        # R : Reset
        elif c == 'R':
            self.reset_requested = True
            self.cmd_vel[:] = 0.0
            self._print_status("Reset Robot Simulation")

    def _print_status(self, action_str=""):
        vx, vy, wz = self.cmd_vel
        sys.stdout.write(f"\r\033[K🕹️  [{self.state:<7}] Cmd: [vx={vx:+5.2f} m/s, vy={vy:+5.2f} m/s, wz={wz:+5.2f} rad/s] | {action_str}\n")
        sys.stdout.flush()


# ==============================================================================
# 3. MAIN SIM-TO-SIM SIMULATOR (ISAAC LAB + VISER)
# ==============================================================================
def main():
    # Parse Custom CLI Arguments
    parser = argparse.ArgumentParser(description="NXP Jaguar Isaac Lab Sim-to-Sim with Viser Visualizer & Teleop")
    parser.add_argument("--task", type=str, default="rough", choices=["rough", "flat"], help="Task environment ('rough' or 'flat')")
    parser.add_argument("--num_envs", type=int, default=1, help="Number of robot environments to spawn (default: 1)")
    parser.add_argument("--load_run", type=str, default=None, help="Specific run timestamp to load")
    parser.add_argument("--policy", type=str, default=None, help="Direct path to TorchScript policy.pt")

    from isaaclab.app import AppLauncher
    AppLauncher.add_app_launcher_args(parser)
    args_cli, remaining_args = parser.parse_known_args()

    # Default to 1 environment and Viser visualizer
    if not hasattr(args_cli, "num_envs") or args_cli.num_envs is None:
        args_cli.num_envs = 1
    if not hasattr(args_cli, "visualizer") or args_cli.visualizer is None:
        args_cli.visualizer = "viser"

    # Launch Simulation App
    app_launcher = AppLauncher(args_cli)
    simulation_app = app_launcher.app

    # Imports AFTER SimulationApp is launched
    import torch
    import gymnasium as gym
    import isaaclab_tasks
    from isaaclab_tasks.utils.parse_cfg import parse_env_cfg

    # Resolve task name
    task_map = {
        "rough": "Isaac-Velocity-Rough-NXP-Jaguar-v0",
        "flat": "Isaac-Velocity-Flat-NXP-Jaguar-v0",
    }
    task_name = task_map.get(args_cli.task, "Isaac-Velocity-Rough-NXP-Jaguar-v0")

    # Load JIT Policy
    policy_path = find_latest_policy(args_cli.policy, args_cli.load_run, args_cli.task)
    print(f"\n[INFO] Loading JIT Policy: {policy_path}\n")
    policy = torch.jit.load(policy_path, map_location="cuda:0")
    policy.eval()

    # Parse Environment Config (1 single robot env)
    env_cfg = parse_env_cfg(task_name, device="cuda:0", num_envs=args_cli.num_envs)
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.scene.env_spacing = 2.5
    env_cfg.observations.policy.enable_corruption = False

    # Disable randomizations on reset so robot spawns cleanly in nominal standing posture
    if hasattr(env_cfg, "events"):
        env_cfg.events.reset_robot_joints = None
        env_cfg.events.push_robot = None
        env_cfg.events.base_external_force_torque = None

    # Configure Camera Auto-Follow (Follow Robot Base Root)
    if hasattr(env_cfg, "viewer"):
        env_cfg.viewer.origin_type = "asset_root"
        env_cfg.viewer.asset_name = "robot"
        env_cfg.viewer.eye = (-2.0, -2.0, 1.2)
        env_cfg.viewer.lookat = (0.0, 0.0, 0.3)
        print("[INFO] Camera Auto-Follow Enabled: Tracking robot asset_root.")

    # Create Gym Environment
    env = gym.make(task_name, cfg=env_cfg)

    # Initialize Teleop Controller
    teleop = TeleopController()
    term_input = TerminalInputHandler(teleop)
    term_input.start()

    print("=" * 75)
    print(" 🐾 NXP JAGUAR: ISAAC LAB 3.0 SIM-TO-SIM (VISER VISUALIZER)")
    print("=" * 75)
    print(f" [INFO] Running 1 Robot on {args_cli.task.upper()} Terrain with Viser 3D WebGL server")
    print(" 🕹️  KEYBOARD CONTROLS (Ketik di Terminal):")
    print("    [1]         : Set State to STANDBY (Duduk / Istirahat)")
    print("    [2]         : Set State to STANDUP (Berdiri sudut q0)")
    print("    [3]         : Set State to WALK    (Aktifkan RL Policy)")
    print("    [W / S]     : Maju / Mundur        (Linear Velocity X)")
    print("    [A / D]     : Geser Kiri / Kanan   (Linear Velocity Y)")
    print("    [Q / E]     : Putar Kiri / Kanan   (Angular Velocity Yaw)")
    print("    [Space]     : Berhenti             (Zero Velocity)")
    print("    [R]         : Reset Robot")
    print("=" * 75)
    print(" Buka link Viser di browser Anda (misal: http://127.0.0.1:8080) untuk melihat 3D!\n")

    # Reset environment
    obs, _ = env.reset()

    # Bind Teleop Commands directly to Isaac Lab Command Manager
    if hasattr(env.unwrapped, "command_manager"):
        cmd_term = env.unwrapped.command_manager.get_term("base_velocity")
        if cmd_term is not None:
            cmd_term.cfg.heading_command = False
            if hasattr(cmd_term, "is_standing_env"):
                cmd_term.is_standing_env[:] = False
            if hasattr(cmd_term, "is_heading_env"):
                cmd_term.is_heading_env[:] = False

            def teleop_resample(env_ids):
                cmd_term.vel_command_b[env_ids, 0] = float(teleop.cmd_vel[0])
                cmd_term.vel_command_b[env_ids, 1] = float(teleop.cmd_vel[1])
                cmd_term.vel_command_b[env_ids, 2] = float(teleop.cmd_vel[2])

            def teleop_update():
                cmd_term.vel_command_b[:, 0] = float(teleop.cmd_vel[0])
                cmd_term.vel_command_b[:, 1] = float(teleop.cmd_vel[1])
                cmd_term.vel_command_b[:, 2] = float(teleop.cmd_vel[2])

            cmd_term._resample_command = teleop_resample
            cmd_term._update_command = teleop_update

    try:
        while simulation_app.is_running():
            if teleop.reset_requested:
                obs, _ = env.reset()
                teleop.reset_requested = False

            # Extract policy observations
            if isinstance(obs, dict):
                policy_obs = obs["policy"]
            else:
                policy_obs = obs

            # Directly inject teleoperation command into observation tensor slice [9:12]
            policy_obs[:, 9] = float(teleop.cmd_vel[0])
            policy_obs[:, 10] = float(teleop.cmd_vel[1])
            policy_obs[:, 11] = float(teleop.cmd_vel[2])

            # State Machine: STANDUP (Hold q0) vs WALK (DreamWaQ RL Policy)
            if teleop.state in ["STANDBY", "STANDUP"]:
                actions = torch.zeros((args_cli.num_envs, 12), device="cuda:0", dtype=torch.float32)
            elif teleop.state == "WALK":
                with torch.no_grad():
                    actions = policy(policy_obs)

            # Step Environment
            obs, rewards, dones, truncated, info = env.step(actions)

    finally:
        term_input.stop()
        env.close()
        simulation_app.close()


if __name__ == "__main__":
    main()
