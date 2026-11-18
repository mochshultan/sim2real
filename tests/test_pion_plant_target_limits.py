# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""The MuJoCo plant must reproduce the CAN driver's extra mechanical target guard."""

from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))
from pion_mujoco_plant import PionMujocoPlant


def test_plant_applies_driver_limits_to_targets_without_modifying_feedback():
    plant = PionMujocoPlant(Path(__file__).parents[1] / "sim2sim/pion_current/pion_qr1.xml")
    before = plant.feedback()[0]
    target = [0.8, -0.8, 0.1, -0.1, -4.0, 4.0, -1.25, -0.8, -1.0, 4.0, 0.5, 1.2]
    plant.set_command(target, [0.0] * 12, [45.0] * 12, [4.5] * 12)
    np.testing.assert_allclose(plant.target, [0.7, -0.7, 0.1, -0.1, -2.0, 1.0, -1.25, -0.8, -0.05, 1.5, 0.5, 1.2])
    np.testing.assert_array_equal(plant.feedback()[0], before)
