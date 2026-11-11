# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Exercise both policy coordinate frames without ROS or hardware."""

import ast
import hashlib
import json
from pathlib import Path

import numpy as np
import pytest
import torch


ROOT = Path(__file__).parents[1]
source = ast.parse((ROOT / "scripts/nxp_jaguar_controller.py").read_text())
builder_class = next(
    node for node in source.body if isinstance(node, ast.ClassDef) and node.name == "JaguarObservationBuilder"
)
nominal = np.array([0.0] * 4 + [-1.51, -1.51, -1.49, -1.49] + [1.22, 1.22, 1.20, 1.20], dtype=np.float32)
namespace = {"np": np, "torch": torch, "DEFAULT_JOINT_POS": nominal, "ACTION_SCALE": 0.25}
exec(compile(ast.Module(body=[builder_class], type_ignores=[]), "observation-builder", "exec"), namespace)
Builder = namespace["JaguarObservationBuilder"]


@pytest.mark.parametrize("reference", [None, np.zeros(12, dtype=np.float32)])
def test_observation_and_target_use_the_same_reference(reference):
    builder = Builder(joint_reference=reference)
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
        Builder(joint_reference=reference)


def test_reference_is_copied_and_does_not_modify_the_default():
    reference = np.zeros(12, dtype=np.float32)
    builder = Builder(joint_reference=reference)
    reference[:] = 1.0
    np.testing.assert_array_equal(builder.joint_reference, np.zeros(12))
    np.testing.assert_array_equal(Builder().joint_reference, nominal)


@pytest.mark.parametrize("tag", [
    "adaboot_ideal_l2c2_zero_reference_5999_20261103",
    "adaboot_l2c2_zero_reference_clearance5cm_19999_20261111",
])
def test_zero_artifacts_match_the_deployment_manifest(tag):
    models = ROOT / "models"
    manifest = json.loads((models / f"deployment_{tag}.json").read_text())
    for kind in ("policy", "checkpoint"):
        digest = hashlib.sha256((models / manifest[f"{kind}_file"]).read_bytes()).hexdigest()
        assert digest == manifest[f"{kind}_sha256"]
    assert manifest["policy_reference"] == "zero"
    assert manifest["nominal_joint_pos_rad"] == [0.0] * 12


def test_repeat_history_matches_training_cold_start_and_clears_previous_action():
    builder = Builder(joint_reference=np.zeros(12), history_initialization="repeat")
    builder.last_action[:] = 8.0
    initial = np.arange(45, dtype=np.float32) / 10.0
    builder.reset_history(initial)
    expected = initial.copy()
    expected[33:45] = 0.0
    np.testing.assert_allclose(builder.history_buf, np.tile(expected, (1, 5, 1)))
    np.testing.assert_array_equal(builder.last_action, np.zeros(12))
    next_step = np.ones(45, dtype=np.float32)
    history = builder.update_and_get_history(next_step).numpy()
    np.testing.assert_allclose(history[0, :4], np.tile(expected, (4, 1)))
    np.testing.assert_allclose(history[0, 4], next_step)


def test_invalid_history_initialization_is_rejected():
    with pytest.raises(ValueError, match="history_initialization"):
        Builder(history_initialization="invalid")
