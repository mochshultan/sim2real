# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Exercise both policy coordinate frames without ROS or hardware."""

import hashlib
import json
from pathlib import Path
import sys

import numpy as np
import pytest
import torch


ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from policy_observation import JaguarObservationBuilder as Builder

nominal = np.array([0.0] * 4 + [-1.51, -1.51, -1.49, -1.49] + [1.22, 1.22, 1.20, 1.20], dtype=np.float32)


@pytest.mark.parametrize("reference", [None, np.zeros(12, dtype=np.float32)])
def test_observation_and_target_use_the_same_reference(reference):
    builder = Builder(nominal if reference is None else reference)
    position = nominal.copy()
    action = np.linspace(-2.0, 2.0, 12, dtype=np.float32)
    origin = nominal if reference is None else reference
    observation = builder.build_step_observation(
        np.zeros(3), [0.0, 0.0, 0.0, 1.0], np.zeros(3), position, np.zeros(12)
    )
    np.testing.assert_allclose(observation[9:21], position - origin)
    np.testing.assert_allclose(builder.build_joint_target(action), origin + 0.25 * action)
    np.testing.assert_allclose(builder.build_joint_target(np.zeros(12)), origin)


@pytest.mark.parametrize("reference", [np.zeros(11), np.full(12, np.nan), np.full(12, np.inf)])
def test_invalid_reference_is_rejected(reference):
    with pytest.raises(ValueError, match="12 finite"):
        Builder(reference)


def test_reference_is_copied_and_does_not_modify_the_default():
    reference = np.zeros(12, dtype=np.float32)
    builder = Builder(reference)
    reference[:] = 1.0
    np.testing.assert_array_equal(builder.default_joint_pos, np.zeros(12))
    np.testing.assert_array_equal(Builder(nominal).default_joint_pos, nominal)


def test_zero_artifacts_match_the_deployment_manifest():
    tag = "adaboot_ideal_l2c2_zero_reference_5999_20261103"
    models = ROOT / "models"
    manifest = json.loads((models / f"deployment_{tag}.json").read_text())
    for kind in ("policy", "checkpoint"):
        digest = hashlib.sha256((models / manifest[f"{kind}_file"]).read_bytes()).hexdigest()
        assert digest == manifest[f"{kind}_sha256"]
    assert manifest["policy_reference"] == "zero"
    assert manifest["nominal_joint_pos_rad"] == [0.0] * 12


@pytest.mark.parametrize("batch", [1, 16, 64])
def test_zero_policy_accepts_temporal_observations(batch):
    policy = torch.jit.load(str(ROOT / "models/policy_adaboot_ideal_l2c2_zero_reference_5999_20261103.pt"), map_location="cpu").eval()
    with torch.no_grad():
        result = policy(torch.zeros((batch, 5, 45), dtype=torch.float32))
    assert tuple(result.shape) == (batch, 12)
    assert torch.isfinite(result).all()
