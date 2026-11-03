# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Prepare a Pion DreamWaQ export for the Jaguar controller joint convention."""

import argparse
import hashlib
import json
from pathlib import Path
import shutil

import torch


class PionControllerAdapter(torch.nn.Module):
    """Permute controller FR/FL/BR/BL observations to Pion FL/FR/BL/BR."""

    def __init__(self, policy: torch.nn.Module):
        super().__init__()
        self.policy = policy
        joint_order = torch.tensor([1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10])
        obs_order = torch.cat((torch.arange(9), joint_order + 9, joint_order + 21, joint_order + 33))
        self.register_buffer("obs_order", obs_order)
        self.register_buffer("joint_order", joint_order)

    def forward(self, history: torch.Tensor) -> torch.Tensor:
        """Return controller-order actions for a batch of five 45-D frames."""
        return self.policy(history.index_select(-1, self.obs_order)).index_select(-1, self.joint_order)


def sha256(path: Path) -> str:
    """Return a file's SHA-256 digest."""
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def main() -> None:
    """Archive the checkpoint, export an adapter, and validate its contract."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    args = parser.parse_args()
    models = Path(__file__).resolve().parents[1] / "models"
    name = "pion_dreamwaq_oldnominal_9999_20261028"
    source = args.run_dir.resolve()
    checkpoint = torch.load(source / "model_9999.pt", map_location="cpu", weights_only=False)
    raw = torch.jit.load(str(source / "policy_9999.pt"), map_location="cpu").eval()
    assert checkpoint["iteration"] == 9999
    for key, value in raw.state_dict().items():
        assert torch.equal(value, checkpoint["model_state_dict"][key]), key
    adapter = PionControllerAdapter(raw).eval()
    exported = torch.jit.script(adapter)
    torch.manual_seed(42)
    permutation = [1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10]
    max_error = 0.0
    with torch.inference_mode():
        for batch in (1, 16, 64):
            history = torch.randn(batch, 5, 45)
            expected_input = history.clone()
            for start in (9, 21, 33):
                expected_input[..., start:start + 12] = history[..., start:start + 12][..., permutation]
            expected = raw(expected_input)[..., permutation]
            actual = exported(history)
            assert torch.isfinite(actual).all()
            torch.testing.assert_close(actual, expected, rtol=0, atol=0)
            max_error = max(max_error, (actual - expected).abs().max().item())
    default = models / "policy.pt"
    if default.exists():
        backup = models / "backup_model" / f"policy_{sha256(default)}.pt"
        backup.parent.mkdir(exist_ok=True)
        if not backup.exists():
            shutil.copy2(default, backup)
    archive = models / name
    archive.mkdir(exist_ok=True)
    for filename in ("env.yaml", "agent.yaml"):
        shutil.copy2(source / "params" / filename, archive / filename)
    shutil.copy2(source / "input_summary.txt", archive / "input_summary.txt")
    shutil.copy2(source / "model_9999.pt", models / f"model_{name}.pt")
    shutil.copy2(source / "policy_9999.pt", archive / "policy_pion_order.pt")
    named = models / f"policy_{name}.pt"
    exported.save(str(named))
    shutil.copy2(named, default)
    manifest = {
        "source_run": str(source), "iteration": 9999,
        "selection_metric": "AdaptiveEntropy/success_rate_ema at final iteration",
        "selection_value": float(checkpoint["adaptive_entropy_state"]["success_rate_ema"]),
        "selection_scope": "six completed old-nominal Pion 10k runs",
        "input_shape": ["batch", 5, 45], "output_shape": ["batch", 12],
        "controller_leg_order": ["FR", "FL", "BR", "BL"],
        "training_leg_order": ["FL", "FR", "BL", "BR"],
        "joint_groups": ["collar", "hip", "knee"],
        "joint_permutation": permutation,
        "nominal_joint_pos_rad": [0.0] * 4 + [-1.0] * 4 + [1.0] * 4,
        "action_scale": 0.25, "kp": 45.0, "kd": 4.5, "control_dt_s": 0.02,
        "raw_export_state_matches_checkpoint": True,
        "adapter_max_abs_error": max_error,
        "checkpoint_sha256": sha256(models / f"model_{name}.pt"),
        "raw_policy_sha256": sha256(archive / "policy_pion_order.pt"),
        "policy_sha256": sha256(named),
        "env_sha256": sha256(archive / "env.yaml"),
        "agent_sha256": sha256(archive / "agent.yaml"),
        "validation_scope": "offline state equality and randomized permutation parity; no hardware motion",
        "hardware_torque_limit_nm": 13.5, "training_torque_limit_nm": 14.0,
    }
    (models / f"deployment_{name}.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()
