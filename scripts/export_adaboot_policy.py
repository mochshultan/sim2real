#!/usr/bin/env python3
"""Export the Jaguar AdaBoot actor for CPU sim-to-real inference."""

from __future__ import annotations

import argparse
import math
import os
import shutil
from pathlib import Path

import torch
import torch.nn as nn


class AdaBootPolicy(nn.Module):
    """TorchScript-compatible estimator-only actor.

    The runtime input is time-major ``(batch, 5, 45)``.  Isaac Lab's
    observation manager stores history per observation term, so the wrapper
    converts it to ``[15, 15, 15, 60, 60, 60]`` before the estimator.
    """

    def __init__(self, actor_state_dict: dict[str, torch.Tensor], eps: float = 0.01, clip_actions: float = 0.0) -> None:
        super().__init__()
        self.register_buffer("mean", actor_state_dict["obs_normalizer._mean"].clone().cpu())
        self.register_buffer("std", actor_state_dict["obs_normalizer._std"].clone().cpu())
        self.eps = eps
        if not math.isfinite(clip_actions) or clip_actions < 0:
            raise ValueError("clip_actions must be finite and nonnegative (zero disables clipping)")
        self.clip_actions = clip_actions

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
        action = self.actor((latent - self.mean) / (self.std + self.eps))
        return action.clamp(-self.clip_actions, self.clip_actions) if self.clip_actions > 0 else action


class PionJointOrderPolicy(nn.Module):
    """Present Pion's FL/FR/BL/BR actor through the controller's FR/FL/BR/BL API.

    Position, velocity and previous-action blocks in every history frame are
    reordered before estimator history packing. Body-frame IMU and command fields
    keep their original order. Output actions are mapped back to the controller.
    """

    def __init__(self, policy: nn.Module) -> None:
        super().__init__()
        self.policy = policy
        joint_order = torch.tensor([1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10], dtype=torch.long)
        self.register_buffer("action_order", joint_order)
        self.register_buffer(
            "observation_order",
            torch.cat((torch.arange(9), joint_order + 9, joint_order + 21, joint_order + 33)),
        )

    def forward(self, history: torch.Tensor) -> torch.Tensor:
        pion_actions = self.policy(history.index_select(-1, self.observation_order))
        return pion_actions.index_select(-1, self.action_order)


def main() -> None:
    parser = argparse.ArgumentParser(description="Export AdaBoot estimator-only policy for Jaguar sim-to-real")
    parser.add_argument(
        "--checkpoint",
        required=True,
    )
    parser.add_argument("--models-dir", default=str(Path(__file__).resolve().parents[1] / "models"))
    parser.add_argument("--tag", default="adaboot_tibia_2999_estimator")
    parser.add_argument("--clip-actions", type=float, default=0.0, help="Clip exported mean actions; zero keeps unbounded output")
    parser.add_argument("--set-default", action="store_true", help="Also replace models/policy.pt")
    parser.add_argument(
        "--training-joint-order", choices=("jaguar", "pion"), default="jaguar",
        help="Adapt the training order to the existing controller without changing hardware mappings",
    )
    args = parser.parse_args()

    checkpoint = os.path.abspath(args.checkpoint)
    if not os.path.isfile(checkpoint):
        raise FileNotFoundError(checkpoint)
    checkpoint_data = torch.load(checkpoint, map_location="cpu", weights_only=False)
    actor_state_dict = checkpoint_data["actor_state_dict"]
    model = AdaBootPolicy(actor_state_dict, clip_actions=args.clip_actions).eval()
    if args.training_joint_order == "pion":
        model = PionJointOrderPolicy(model).eval()

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
