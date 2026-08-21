"""Observation Builder for NXP Jaguar 48-D DreamWaQ Policy."""

import numpy as np
import torch

# Isaac Lab 3.0 Joint Order (Rolls, Hips, Knees)
ISAAC_JOINT_NAMES = [
    "Fr_roll_joint", "Fl_roll_joint", "Br_roll_joint", "Bl_roll_joint",
    "Fr_hip_pitch_joint", "Fl_hip_pitch_joint", "Br_hip_pitch_joint", "Bl_hip_pitch_joint",
    "Fr_knee_joint", "Fl_knee_joint", "Br_knee_joint", "Bl_knee_joint",
]

# MuJoCo Actuator Order (FR, FL, BR, BL)
MUJOCO_JOINT_NAMES = [
    "Fr_roll_joint", "Fr_hip_pitch_joint", "Fr_knee_joint",
    "Fl_roll_joint", "Fl_hip_pitch_joint", "Fl_knee_joint",
    "Br_roll_joint", "Br_hip_pitch_joint", "Br_knee_joint",
    "Bl_roll_joint", "Bl_hip_pitch_joint", "Bl_knee_joint",
]

# Remapping Indices: MuJoCo Order (FR, FL, BR, BL) <-> Isaac Lab Order (Rolls, Hips, Knees)
# Isaac: [FR_r, FL_r, BR_r, BL_r, FR_h, FL_h, BR_h, BL_h, FR_k, FL_k, BR_k, BL_k]
MUJOCO_TO_ISAAC = [0, 3, 6, 9, 1, 4, 7, 10, 2, 5, 8, 11]
ISAAC_TO_MUJOCO = [0, 4, 8, 1, 5, 9, 2, 6, 10, 3, 7, 11]

# Nominal Standing Angles (q0) in Isaac Lab Joint Order (FR, FL, BR, BL)
DEFAULT_JOINT_POS_ISAAC = np.array([
    0.0,   0.0,   0.0,   0.0,    # Rolls (FR, FL, BR, BL)
   -1.50, -1.50, -1.40, -1.40,   # Hips (FR, FL, BR, BL)
    1.40,  1.40,  1.36,  1.36,   # Knees (FR, FL, BR, BL)
], dtype=np.float32)

def quat_rotate_inverse(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotates world frame vector v into body frame using quaternion q = [w, x, y, z]."""
    w, x, y, z = q[0], q[1], q[2], q[3]
    q_vec = np.array([x, y, z], dtype=np.float32)
    a = v * (2.0 * w ** 2 - 1.0)
    b = np.cross(q_vec, v) * w * 2.0
    c = q_vec * (np.dot(q_vec, v)) * 2.0
    return (a - b + c).astype(np.float32)

class ObservationBuilder:
    def __init__(self):
        self.last_action = torch.zeros((1, 12), dtype=torch.float32)

    def build_observation(
        self,
        lin_vel: np.ndarray,
        ang_vel: np.ndarray,
        quat: np.ndarray,
        cmd: np.ndarray,
        joint_pos_isaac: np.ndarray,
        joint_vel_isaac: np.ndarray,
    ) -> torch.Tensor:
        """
        Builds exact 48D Observation Vector for DreamWaQ Actor:
        [0:3]   Linear Velocity (base frame)
        [3:6]   Angular Velocity (base frame)
        [6:9]   Projected Gravity Vector
        [9:12]  Velocity Commands [vx, vy, wz]
        [12:24] Relative Joint Positions (joint_pos - default_pos)
        [24:36] Joint Velocities
        [36:48] Last Action
        """
        # Gravity projection in body frame (R^T * [0, 0, -1])
        proj_gravity = quat_rotate_inverse(quat, np.array([0.0, 0.0, -1.0], dtype=np.float32))

        rel_joint_pos = joint_pos_isaac - DEFAULT_JOINT_POS_ISAAC

        obs_vec = np.concatenate([
            lin_vel,                     # 3D: vx, vy, vz (body frame)
            ang_vel,                     # 3D: wx, wy, wz (body frame)
            proj_gravity,                # 3D: projected gravity (body frame)
            cmd,                         # 3D: vx_cmd, vy_cmd, wz_cmd
            rel_joint_pos,               # 12D: q - q0 (Isaac order)
            joint_vel_isaac,             # 12D: q_dot (Isaac order)
            self.last_action[0].numpy(), # 12D: a_{t-1}
        ], axis=0).astype(np.float32)

        return torch.from_numpy(obs_vec).unsqueeze(0)

    def update_last_action(self, action_tensor: torch.Tensor):
        self.last_action = action_tensor.clone()
