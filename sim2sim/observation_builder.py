"""Observation Builder for NXP Jaguar 45-D DreamWaQ CENet Policy."""

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
MUJOCO_TO_ISAAC = [0, 3, 6, 9, 1, 4, 7, 10, 2, 5, 8, 11]
ISAAC_TO_MUJOCO = [0, 4, 8, 1, 5, 9, 2, 6, 10, 3, 7, 11]

DEFAULT_JOINT_POS_ISAAC = np.array([
    0.0,   0.0,   0.0,   0.0,    # Rolls (FR, FL, BR, BL)
   -1.55, -1.55, -1.45, -1.45,   # Hips  (FR, FL, BR, BL)
    1.42,  1.42,  1.35,  1.35,   # Knees (FR, FL, BR, BL)
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
    def __init__(self, history_len: int = 5):
        self.history_len = history_len
        self.obs_dim = 45
        self.last_action = np.zeros(12, dtype=np.float32)
        # History buffer: shape (1, 5, 45)
        self.history_buf = np.zeros((1, history_len, self.obs_dim), dtype=np.float32)

    def reset_history(self, initial_obs_45d: np.ndarray):
        for i in range(self.history_len):
            self.history_buf[0, i, :] = initial_obs_45d
        self.last_action[:] = 0.0

    def build_step_observation(
        self,
        ang_vel: np.ndarray,
        quat: np.ndarray,
        cmd: np.ndarray,
        joint_pos_isaac: np.ndarray,
        joint_vel_isaac: np.ndarray,
    ) -> np.ndarray:
        """
        Builds exact 45D Proprioceptive Observation Vector:
        [0:3]   Angular Velocity (base frame gyro)
        [3:6]   Projected Gravity Vector
        [6:9]   Velocity Commands [vx, vy, wz]
        [9:21]  Relative Joint Positions (joint_pos - default_pos)
        [21:33] Joint Velocities
        [33:45] Last Action
        """
        # Gravity projection in body frame (R^T * [0, 0, -1])
        proj_gravity = quat_rotate_inverse(quat, np.array([0.0, 0.0, -1.0], dtype=np.float32))
        rel_joint_pos = joint_pos_isaac - DEFAULT_JOINT_POS_ISAAC

        obs_45d = np.concatenate([
            ang_vel,                     # 3D: wx, wy, wz (body frame)
            proj_gravity,                # 3D: projected gravity (body frame)
            cmd,                         # 3D: vx_cmd, vy_cmd, wz_cmd
            rel_joint_pos,               # 12D: q - q0 (Isaac order)
            joint_vel_isaac,             # 12D: q_dot (Isaac order)
            self.last_action,            # 12D: a_{t-1}
        ], axis=0).astype(np.float32)

        return obs_45d

    def update_and_get_history(self, obs_45d: np.ndarray) -> torch.Tensor:
        self.history_buf = np.roll(self.history_buf, shift=-1, axis=1)
        self.history_buf[0, -1, :] = obs_45d
        return torch.from_numpy(self.history_buf).float()

    def update_last_action(self, action_np: np.ndarray):
        self.last_action = action_np.copy()
