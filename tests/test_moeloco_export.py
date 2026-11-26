# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Native MoE checkpoint/export/trace and isolated PPO launch selection."""

import importlib.util
import sys
from pathlib import Path

import pytest
import torch

ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from export_moeloco_policy import export_policy
from pion_policy import infer_with_trace

BUNDLE = ROOT / "models/history/moeloco_measured_l2c2_5k"
NATIVE = BUNDLE / "policy_pion_qr1_moeloco_measured_native_4999_20261126.pt"


def test_reexport_and_controller_trace_match_training_actor(tmp_path):
    output = tmp_path / "native.pt"
    export_policy(BUNDLE / "model_4999.pt", output)
    native = torch.jit.load(str(output)).eval()
    shipped = torch.jit.load(str(NATIVE)).eval()
    reference = torch.jit.load(str(BUNDLE / "policy.pt")).eval()
    generator = torch.Generator().manual_seed(42)
    with torch.inference_mode():
        for batch, scale in ((1, 0.0), (16, 0.2), (64, 10.0)):
            history = torch.randn(batch, 5, 45, generator=generator) * scale
            packed = torch.cat(
                [history[:, :, a:b].flatten(1) for a, b in ((0, 3), (3, 6), (6, 9), (9, 21), (21, 33), (33, 45))], 1
            )
            expected = reference(history[:, -1], packed).clamp(-10, 10)
            actual = native(history)
            traced, trace = infer_with_trace(native, history)
            torch.testing.assert_close(actual, expected, rtol=0, atol=0)
            torch.testing.assert_close(actual, shipped(history), rtol=0, atol=0)
            torch.testing.assert_close(actual, traced, rtol=0, atol=0)
            assert len(trace["actor_input"]) == 64 and len(trace["estimator_context"]) == 16
            assert torch.isfinite(actual).all()
    with pytest.raises(FileExistsError):
        export_policy(BUNDLE / "model_4999.pt", output)
    ppo = ROOT / "models/history/dreamwaq_vhip_mean_10k/model_9999.pt"
    with pytest.raises(ValueError, match="four-expert"):
        export_policy(ppo, tmp_path / "wrong.pt")


def test_moe_launcher_selects_its_model_without_changing_ppo_default():
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument

    def declarations(name, context):
        path = ROOT / "launch" / name
        spec = importlib.util.spec_from_file_location(name, path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        for action in module.generate_launch_description().entities:
            if isinstance(action, DeclareLaunchArgument):
                action.execute(context)

    moe = LaunchContext()
    declarations("sim2real_moe.launch.py", moe)
    declarations("sim2real_curpos.launch.py", moe)
    assert Path(moe.launch_configurations["policy_path"]) == NATIVE
    assert moe.launch_configurations["actor_input_dim"] == "64"
    ppo = LaunchContext()
    declarations("sim2real_curpos.launch.py", ppo)
    assert Path(ppo.launch_configurations["policy_path"]) != NATIVE
    assert "dreamwaq_vhip_mean_10k" in ppo.launch_configurations["policy_path"]


def test_default_dreamwaq_actor_reexport_preserves_ppo_model():
    from export_adaboot_policy import NativePionPolicy
    from export_dreamwaq_policy import DreamWaQPolicy

    folder = ROOT / "models/history/dreamwaq_vhip_mean_10k"
    data = torch.load(folder / "model_9999.pt", map_location="cpu", weights_only=False)
    regenerated = torch.jit.script(NativePionPolicy(DreamWaQPolicy(data["actor_state_dict"])).eval())
    shipped = torch.jit.load(str(folder / "policy_pion_qr1_dreamwaq_vhip_mean_native_9999_20261125.pt")).eval()
    history = torch.randn(16, 5, 45, generator=torch.Generator().manual_seed(42))
    with torch.inference_mode():
        torch.testing.assert_close(regenerated(history), shipped(history), rtol=0, atol=0)
