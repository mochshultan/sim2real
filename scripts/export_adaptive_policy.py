#!/usr/bin/env python3
"""Export adaptive TorchScript policy for sim2real deployment."""

import os
import torch
import torch.nn as nn

class AdaptiveBaselinePolicy(nn.Module):
    def __init__(self, base_policy: nn.Module):
        super().__init__()
        self.base_policy = base_policy

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        # If 3D temporal history buffer (e.g. [1, 5, 45]), slice latest observation
        if x.dim() == 3:
            obs = x[:, -1, :]
        elif x.dim() == 1:
            obs = x.unsqueeze(0)
        else:
            obs = x
        return self.base_policy(obs)

def main():
    import argparse
    import shutil

    parser = argparse.ArgumentParser(description="Export adaptive TorchScript policy for sim2real")
    parser.add_argument(
        "--raw-policy",
        type=str,
        default="/home/shultan/IsaacLab/logs/rsl_rl/nxp_jaguar_baseline_tibia/2026-09-15_10-46-44/exported/policy.pt",
        help="Path to raw TorchScript policy exported from Isaac Lab / RSL-RL",
    )
    parser.add_argument(
        "--tag",
        type=str,
        default="baseline_tibia_2500",
        help="Model tag for versioned output filename",
    )
    parser.add_argument(
        "--models-dir",
        type=str,
        default="/home/shultan/jaguar_sim2real/models",
        help="Target models directory",
    )
    parser.add_argument(
        "--set-default",
        action="store_true",
        default=True,
        help="Also overwrite policy.pt and policy.onnx as default",
    )
    args = parser.parse_args()

    os.makedirs(args.models_dir, exist_ok=True)
    raw_policy = torch.jit.load(args.raw_policy, map_location="cpu")
    raw_policy.eval()

    wrapper = AdaptiveBaselinePolicy(raw_policy)
    scripted_policy = torch.jit.script(wrapper)
    scripted_policy.eval()

    # Verification tests
    dummy_3d = torch.randn(1, 5, 45)
    dummy_2d = dummy_3d[:, -1, :].clone()
    dummy_1d = dummy_2d.squeeze(0).clone()

    out_raw = raw_policy(dummy_2d)
    out_3d = scripted_policy(dummy_3d)
    out_2d = scripted_policy(dummy_2d)
    out_1d = scripted_policy(dummy_1d)

    print("out_raw shape:", out_raw.shape)
    print("out_3d shape:", out_3d.shape)
    print("out_2d shape:", out_2d.shape)
    print("out_1d shape:", out_1d.shape)

    assert (out_raw - out_3d).abs().max().item() < 1e-6, "3D output mismatch!"
    assert (out_raw - out_2d).abs().max().item() < 1e-6, "2D output mismatch!"
    assert (out_raw - out_1d).abs().max().item() < 1e-6, "1D output mismatch!"

    versioned_path = os.path.join(args.models_dir, f"policy_{args.tag}.pt")
    torch.jit.save(scripted_policy, versioned_path)
    print(f"[SUCCESS] Exported adaptive policy to: {versioned_path}")

    if args.set_default:
        default_path = os.path.join(args.models_dir, "policy.pt")
        torch.jit.save(scripted_policy, default_path)
        print(f"[SUCCESS] Updated active default: {default_path}")

        # Also copy ONNX files if present alongside raw policy
        raw_dir = os.path.dirname(args.raw_policy)
        raw_onnx = os.path.join(raw_dir, "policy.onnx")
        raw_onnx_data = os.path.join(raw_dir, "policy.onnx.data")
        if os.path.exists(raw_onnx):
            shutil.copy2(raw_onnx, os.path.join(args.models_dir, f"policy_{args.tag}.onnx"))
            shutil.copy2(raw_onnx, os.path.join(args.models_dir, "policy.onnx"))
            print(f"[SUCCESS] Updated policy_{args.tag}.onnx and policy.onnx")
        if os.path.exists(raw_onnx_data):
            shutil.copy2(raw_onnx_data, os.path.join(args.models_dir, f"policy_{args.tag}.onnx.data"))
            shutil.copy2(raw_onnx_data, os.path.join(args.models_dir, "policy.onnx.data"))
            print(f"[SUCCESS] Updated policy_{args.tag}.onnx.data and policy.onnx.data")

if __name__ == "__main__":
    main()
