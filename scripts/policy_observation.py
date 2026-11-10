"""Policy observation scaling and temporal history, independent of ROS and CAN."""

import numpy as np
import torch


class JaguarObservationBuilder:
    def __init__(self, default_joint_pos, history_len: int = 5):
        self.default_joint_pos = np.asarray(default_joint_pos, dtype=np.float32).copy()
        if self.default_joint_pos.shape != (12,) or not np.isfinite(self.default_joint_pos).all():
            raise ValueError("Joint reference must contain 12 finite positions [rad]")
        self.history_len = history_len
        self.obs_dim = 45
        self.last_action = np.zeros(12, dtype=np.float32)
        # Temporal history ring buffer: shape (1, 5, 45)
        self.history_buf = np.zeros((1, history_len, self.obs_dim), dtype=np.float32)

    def reset_history(self, initial_obs_45d: np.ndarray):
        self.history_buf.fill(0.0)
        self.history_buf[0, -1, :] = initial_obs_45d
        self.last_action[:] = 0.0

    def build_step_observation(self, ang_vel, quat, cmd, joint_pos, joint_vel) -> np.ndarray:
        """
        Builds 45D proprioceptive observation vector:
        [0:3]   Angular Velocity (base frame gyro)
        [3:6]   Projected Gravity Vector
        [6:9]   Velocity Commands [vx, vy, wz]
        [9:21]  Relative Joint Positions (joint_pos - default_pos)
        [21:33] Joint Velocities
        [33:45] Last Action
        """
        qx, qy, qz, qw = quat
        # Gravity projection in body frame (R^T * [0, 0, -1])
        gx = -2.0 * (qx * qz - qw * qy)
        gy = -2.0 * (qy * qz + qw * qx)
        gz = -(1.0 - 2.0 * (qx * qx + qy * qy))

        rel_joint_pos = joint_pos - self.default_joint_pos

        obs_45d = np.concatenate([
            ang_vel * 0.25,              # DreamWaQ angular velocity scale
            [gx, gy, gz],                # 3D: projected gravity
            cmd * np.array([2.0, 2.0, 0.25], dtype=np.float32),
                                         # DreamWaQ command scales
            rel_joint_pos,               # 12D: q - q0 (scale 1.0)
            joint_vel * 0.05,            # DreamWaQ joint velocity scale
            self.last_action,            # 12D: a_{t-1}
        ], axis=0).astype(np.float32)

        return obs_45d

    def build_joint_target(self, action: np.ndarray, action_scale: float = 0.25) -> np.ndarray:
        """Use the observation reference to convert policy actions to joint targets."""
        return self.default_joint_pos + action_scale * action

    def update_and_get_history(self, obs_45d: np.ndarray) -> torch.Tensor:
        # Shift history left and insert newest observation at the end
        self.history_buf = np.roll(self.history_buf, shift=-1, axis=1)
        self.history_buf[0, -1, :] = obs_45d
        return torch.from_numpy(self.history_buf).float()

    def update_last_action(self, action_np: np.ndarray):
        self.last_action = action_np.copy()
