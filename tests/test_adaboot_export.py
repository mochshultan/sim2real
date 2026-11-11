# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Exported action limits must be applied before controller target/history feedback."""

import importlib.util
from pathlib import Path

import torch

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("adaboot_export", ROOT / "scripts/export_adaboot_policy.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_export_clips_actions_without_changing_legacy_unbounded_contract():
    state = torch.load(
        ROOT / "models/model_adaboot_ideal_l2c2_zero_reference_5999_20261103.pt",
        map_location="cpu", weights_only=False,
    )["actor_state_dict"]
    state["mlp.6.weight"].zero_()
    state["mlp.6.bias"].fill_(80.0)
    sample = torch.zeros(1, 5, 45)
    bounded = MODULE.AdaBootPolicy(state, clip_actions=10.0).eval()
    legacy = MODULE.AdaBootPolicy(state).eval()
    with torch.inference_mode():
        torch.testing.assert_close(bounded(sample), torch.full((1, 12), 10.0))
        torch.testing.assert_close(legacy(sample), torch.full((1, 12), 80.0))
        scripted = torch.jit.script(bounded)
        torch.testing.assert_close(scripted(sample), torch.full((1, 12), 10.0))
