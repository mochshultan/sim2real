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


def test_pion_export_maps_all_joint_history_fields_and_actions_by_anatomical_name():
    """Catch a leg swap or omitted previous-action/velocity remap at the deployed boundary."""
    tag = "pion_adaboot_l2c2_zero_32v_height25_11500_20261113"
    state = torch.load(ROOT / "models" / f"model_{tag}.pt", map_location="cpu", weights_only=False)["actor_state_dict"]
    training = MODULE.AdaBootPolicy(state, clip_actions=10.0).eval()
    deployed = torch.jit.load(str(ROOT / "models" / f"policy_{tag}.pt"), map_location="cpu").eval()
    runtime_names = [f"{leg}_{joint}" for joint in ("collar", "hip", "knee") for leg in ("FR", "FL", "BR", "BL")]
    training_names = [f"{leg}_{joint}" for joint in ("collar", "hip", "knee") for leg in ("FL", "FR", "BL", "BR")]
    torch.manual_seed(42)
    history = torch.randn(4, 5, 45) * 0.3
    # Independent name lookup specifies the physical relationship, not adapter buffers.
    fields = [history[..., :9]]
    for start in (9, 21, 33):
        by_name = {name: history[..., start + index] for index, name in enumerate(runtime_names)}
        fields.append(torch.stack([by_name[name] for name in training_names], -1))
    with torch.inference_mode():
        pion_actions = training(torch.cat(fields, -1))
        by_name = {name: pion_actions[:, index] for index, name in enumerate(training_names)}
        expected = torch.stack([by_name[name] for name in runtime_names], -1)
        torch.testing.assert_close(deployed(history), expected, rtol=0, atol=1e-6)
