# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Native Pion Current observation/action contract and named hardware boundary."""

import numpy as np

from policy_observation import JaguarObservationBuilder

JOINT_NAMES = tuple(
    f"{leg}_{axis}_joint"
    for axis in ("collar", "hip", "knee")
    for leg in ("FL", "FR", "BL", "BR")
)
# Indices of native policy joints within the hardware BL/BR/FL/FR leg-major packet.
ROS_TO_PION = (6, 9, 0, 3, 7, 10, 1, 4, 8, 11, 2, 5)
NAME_TO_PION = {name: index for index, name in enumerate(JOINT_NAMES)}
for index, name in enumerate(JOINT_NAMES):
    leg, axis, _ = name.split('_')
    hardware_axis = {"collar": "roll", "hip": "hip_pitch", "knee": "knee"}[axis]
    NAME_TO_PION[f"{leg.title()}_{hardware_axis}_joint"] = index

STANDING = np.array([0.0] * 4 + [-1.25] * 4 + [1.25] * 4, dtype=np.float32)
CONTROL_DT = 0.02
ACTION_SCALE = 0.25
CLIP_ACTIONS = 10.0
KP = 45.0
KD = 4.5


class PionObservationBuilder(JaguarObservationBuilder):
    """45D native proprioception, repeated five-frame history and measured anchor.

    The shared numerical builder uses the training scales (0.25 gyro, 2/2/0.25
    command, 1 joint position, 0.05 joint velocity) independently of joint names.
    Encoder position is relative to zero, not the physical standing pose.
    """

    def __init__(self):
        super().__init__(np.zeros(12, dtype=np.float32), history_len=5,
                         history_initialization="repeat", action_reference="current")
