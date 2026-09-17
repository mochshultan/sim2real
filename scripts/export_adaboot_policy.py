#!/usr/bin/env python3
"""Export the Jaguar AdaBoot actor for CPU sim-to-real inference."""

from __future__ import annotations

import argparse
import os
import shutil

import torch
import torch.nn as nn


class AdaBootPolicy(nn.Module):
    """TorchScript-compatible estimator-only actor.

    The runtime input is time-major ``(batch, 5, 45)``.  Isaac Lab's
    observation manager stores history per observation term, so the wrapper
    converts it to ``[15, 15, 15, 60, 60, 60]`` before the estimator.
    """

    def __init__(self, actor_state_dict: dict[str, torch.Tensor], eps: float = 0.01) -> None:
        super().__init__()
        self.register_buffer("mean", actor_state_dict["obs_normalizer._mean"].clone().cpu())
        self.register_buffer("std", actor_state_dict["obs_normalizer._std"].clone().cpu())
        self.eps = eps

        self.estimator = nn.Sequential(
            nn.Linear(225, 128),
            nn.ELU(),
            nn.Linear(128, 64),
            nn.ELU(),
            nn.Linear(64, 3),
        )
        self.actor = nn.Sequential(
            nn.Linear(48, 512),
            nn.ELU(),
            nn.Linear(512, 256),
            nn.ELU(),
            nn.Linear(256, 128),
            nn.ELU(),
            nn.Linear(128, 12),
        )
        self.estimator.load_state_dict(
            {key.replace("velocity_estimator.", ""): value.cpu() for key, value in actor_state_dict.items() if key.startswith("velocity_estimator.")}
        )
        self.actor.load_state_dict(
            {key.replace("mlp.", ""): value.cpu() for key, value in actor_state_dict.items() if key.startswith("mlp.")}
        )

    def forward(self, history: torch.Tensor) -> torch.Tensor:
        if history.dim() == 2:
            history = history.unsqueeze(0)
        if history.dim() != 3 or history.shape[1] != 5 or history.shape[2] != 45:
            raise RuntimeError("AdaBoot policy expects input shape (batch, 5, 45)")

        current = history[:, -1, :]
        estimator_input = torch.cat(
            (
                history[:, :, 0:3].reshape(-1, 15),
                history[:, :, 3:6].reshape(-1, 15),
                history[:, :, 6:9].reshape(-1, 15),
                history[:, :, 9:21].reshape(-1, 60),
                history[:, :, 21:33].reshape(-1, 60),
                history[:, :, 33:45].reshape(-1, 60),
            ),
            dim=1,
        )
        estimated_velocity = self.estimator(estimator_input)
        latent = torch.cat((current, estimated_velocity), dim=1)
        return self.actor((latent - self.mean) / (self.std + self.eps))


def main() -> None:
    parser = argparse.ArgumentParser(description="Export AdaBoot estimator-only policy for Jaguar sim-to-real")
    parser.add_argument(
        "--checkpoint",
        default="/home/shultan/IsaacLab/logs/rsl_rl/nxp_jaguar_baseline_tibia_adaboot/2026-09-16_20-55-30/model_2999.pt",
    )
    parser.add_argument("--models-dir", default="/home/shultan/jaguar_sim2real/models")
    parser.add_argument("--tag", default="adaboot_tibia_2999_estimator")
    parser.add_argument("--set-default", action="store_true", help="Also replace models/policy.pt")
    args = parser.parse_args()

    checkpoint = os.path.abspath(args.checkpoint)
    if not os.path.isfile(checkpoint):
        raise FileNotFoundError(checkpoint)
    checkpoint_data = torch.load(checkpoint, map_location="cpu", weights_only=False)
    actor_state_dict = checkpoint_data["actor_state_dict"]
    model = AdaBootPolicy(actor_state_dict).eval()

    sample = torch.zeros(1, 5, 45)
    with torch.inference_mode():
        output = model(sample)
    if tuple(output.shape) != (1, 12) or not torch.isfinite(output).all():
        raise RuntimeError(f"Invalid exported output: {tuple(output.shape)}")

    os.makedirs(args.models_dir, exist_ok=True)
    versioned = os.path.join(args.models_dir, f"policy_{args.tag}.pt")
    scripted = torch.jit.script(model)
    torch.jit.save(scripted, versioned)
    checkpoint_copy = os.path.join(args.models_dir, f"model_{args.tag}.pt")
    shutil.copy2(checkpoint, checkpoint_copy)
    print(f"[SUCCESS] Exported estimator-only TorchScript: {versioned}")
    print(f"[INFO] Checkpoint copy: {checkpoint_copy}")

    if args.set_default:
        default_path = os.path.join(args.models_dir, "policy.pt")
        shutil.copy2(versioned, default_path)
        print(f"[SUCCESS] Updated default policy: {default_path}")


if __name__ == "__main__":
    main()
