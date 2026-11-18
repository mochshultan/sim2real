# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Pion dynamics only: no policy, observation history or action-target implementation."""

from pathlib import Path

import mujoco
import numpy as np

PION_CONTROLLER_ORDER = [
    f"{leg}_{joint}_joint" for joint in ("collar", "hip", "knee") for leg in ("FR", "FL", "BR", "BL")
]
STANDING = np.array([0.0] * 4 + [-1.1] * 4 + [1.1] * 4)


def rs00_32v_torque(requested, velocity, cap=14.0):
    """Match the saved training envelope, an unmeasured linear32V approximation."""
    no_load_speed = 315.0 * (32.0 / 48.0) * (2.0 * np.pi / 60.0)
    speed = np.clip(velocity, -2 * no_load_speed, 2 * no_load_speed)
    upper = np.minimum(14.0 * (1.0 - speed / no_load_speed), cap)
    lower = np.maximum(14.0 * (-1.0 - speed / no_load_speed), -cap)
    return np.clip(requested, lower, upper)


class PionMujocoPlant:
    def __init__(self, path, torque_cap=14.0):
        if not np.isfinite(torque_cap) or not 0 < torque_cap <= 14.0:
            raise ValueError("Pion torque cap must be in (0,14] Nm")
        self.torque_cap = float(torque_cap)
        self.model = mujoco.MjModel.from_xml_path(str(Path(path).resolve()))
        self.data = mujoco.MjData(self.model)
        ids = np.array(
            [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in PION_CONTROLLER_ORDER]
        )
        self.actuators = np.array(
            [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in PION_CONTROLLER_ORDER]
        )
        if (ids < 0).any() or (self.actuators < 0).any():
            raise ValueError("Pion model must expose all12 named joints and torque actuators")
        self.qpos_ids = self.model.jnt_qposadr[ids]
        self.qvel_ids = self.model.jnt_dofadr[ids]
        self.torque = np.zeros(12)
        self.target = STANDING.copy()
        self.target_velocity = np.zeros(12)
        self.kp = np.zeros(12)
        self.kd = np.zeros(12)
        self.feedforward = np.zeros(12)
        self.data.qpos[:3] = [0.0, 0.0, 0.22]
        self.data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
        self.data.qpos[self.qpos_ids] = STANDING
        mujoco.mj_forward(self.model, self.data)

    def feedback(self):
        return self.data.qpos[self.qpos_ids].copy(), self.data.qvel[self.qvel_ids].copy(), self.torque.copy()

    def set_command(self, position, velocity, kp, kd, feedforward=None):
        arrays = [np.asarray(value, dtype=float) for value in (position, velocity, kp, kd)]
        if any(value.shape != (12,) or not np.isfinite(value).all() for value in arrays):
            raise ValueError("Impedance commands require12 finite values per field")
        if (arrays[2] < 0).any() or (arrays[3] < 0).any():
            raise ValueError("Impedance gains must be nonnegative")
        self.target, self.target_velocity, self.kp, self.kd = [value.copy() for value in arrays]
        # Match the CAN manager's calibrated-coordinate guards after the shared
        # policy controller has applied its own limits. Do not infer these from
        # MuJoCo physical joint limits: desired PD targets have a separate guard.
        self.target = np.clip(self.target, [-0.7] * 4 + [-2.0] * 4 + [-0.05] * 4,
                              [0.7] * 4 + [1.0] * 4 + [1.6] * 4)
        self.feedforward = np.zeros(12) if feedforward is None else np.asarray(feedforward).copy()
        if self.feedforward.shape != (12,) or not np.isfinite(self.feedforward).all():
            raise ValueError("Feedforward requires12 finite torques")

    def step(self, passive=False):
        position, velocity, _ = self.feedback()
        requested = self.kp * (self.target - position) + self.kd * (self.target_velocity - velocity) + self.feedforward
        self.torque = np.zeros(12) if passive else rs00_32v_torque(requested, velocity, self.torque_cap)
        self.data.ctrl[self.actuators] = self.torque
        mujoco.mj_step(self.model, self.data)
