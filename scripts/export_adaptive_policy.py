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
    raw_policy_path = "/home/shultan/IsaacLab/logs/rsl_rl/nxp_jaguar_baseline/2026-09-14_17-32-43/exported/policy.pt"
    out_policy_path = "/home/shultan/jaguar_sim2real/models/policy.pt"
    out_baseline_3000_path = "/home/shultan/jaguar_sim2real/models/policy_baseline_3000.pt"

    raw_policy = torch.jit.load(raw_policy_path, map_location="cpu")
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

    torch.jit.save(scripted_policy, out_policy_path)
    torch.jit.save(scripted_policy, out_baseline_3000_path)

    print(f"[SUCCESS] Exported adaptive policy to:\n  -> {out_policy_path}\n  -> {out_baseline_3000_path}")

if __name__ == "__main__":
    main()
