#!/usr/bin/env python3
# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Export a feed-forward Pion MoE-Loco policy with CENet19 and no oracle inputs."""

import argparse
from pathlib import Path

import torch
from torch import nn

from export_adaboot_policy import NativePionPolicy
from export_dreamwaq_policy import DreamWaQPolicy


class MoEActor(nn.Module):
    """Blend four action means with the exact normalized-input softmax router."""

    def __init__(self):
        super().__init__()
        self.router = nn.Sequential(nn.Linear(64, 128), nn.ELU(), nn.Linear(128, 64), nn.ELU(), nn.Linear(64, 4))
        self.experts = nn.ModuleList(
            [
                nn.Sequential(
                    nn.Linear(64, 512),
                    nn.ELU(),
                    nn.Linear(512, 256),
                    nn.ELU(),
                    nn.Linear(256, 128),
                    nn.ELU(),
                    nn.Linear(128, 12),
                )
                for _ in range(4)
            ]
        )

    @torch.jit.export
    def weights(self, inputs: torch.Tensor) -> torch.Tensor:
        return self.router(inputs).softmax(dim=-1)

    def forward(self, inputs: torch.Tensor) -> torch.Tensor:
        means = torch.stack([expert(inputs) for expert in self.experts], dim=-2)
        return (self.weights(inputs).unsqueeze(-1) * means).sum(dim=-2)


def export_policy(checkpoint: Path, output: Path) -> None:
    """Validate the checkpoint contract and save the native history-input export."""
    data = torch.load(checkpoint, map_location="cpu", weights_only=False)
    contract = data["pion_baseline"]
    if data["pion_action_contract"] != dict(
        version=1, reference="current_joint_position", scale=0.25, anchor="policy_step"
    ):
        raise ValueError("Expected the Pion current-position action contract")
    required = dict(
        actor_dim=64,
        history_dim=225,
        latent_dim=16,
        estimator="moeloco_cenet19_contact_oracle",
        actor_experts=4,
        critic_experts=4,
        shared_router=True,
        context_mode="mean",
    )
    if any(contract.get(key) != value for key, value in required.items()):
        raise ValueError("Expected a mean-context CENet19 four-expert Pion MoE-Loco checkpoint")
    if output.exists():
        raise FileExistsError(output)
    policy = NativePionPolicy(DreamWaQPolicy(data["actor_state_dict"], actor=MoEActor())).eval()
    scripted = torch.jit.script(policy)
    with torch.inference_mode():
        result = scripted(torch.zeros(1, 5, 45))
        if result.shape != (1, 12) or not torch.isfinite(result).all():
            raise ValueError("Invalid native MoE output")
    output.parent.mkdir(parents=True, exist_ok=True)
    torch.jit.save(scripted, str(output))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    export_policy(args.checkpoint, args.output)
    print(f"Exported complete native MoE policy: {args.output}")


if __name__ == "__main__":
    main()
