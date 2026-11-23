#!/usr/bin/env python3
"""Export a complete deterministic Pion DreamWaQ CENet policy from a checkpoint."""

import argparse
from pathlib import Path

import torch
from torch import nn

from export_adaboot_policy import NativePionPolicy


class DreamWaQPolicy(nn.Module):
    """Existing Pion history convention, normalized CENet and mean context at inference.

    Normalizer epsilon follows the existing deployment convention (0.01).
    Training configuration is required to independently confirm that convention.
    """

    def __init__(self, state):
        super().__init__()
        self.eps = 0.01
        self.clip_actions = 10.0
        for name, key in (("mean", "obs_normalizer._mean"),
                          ("std", "obs_normalizer._std"),
                          ("history_mean", "cenet.history_normalizer._mean"),
                          ("history_std", "cenet.history_normalizer._std")):
            self.register_buffer(name, state[key].clone().cpu())
        self.encoder = nn.Sequential(nn.Linear(225, 128), nn.ELU(),
                                     nn.Linear(128, 64), nn.ELU())
        self.velocity_head = nn.Linear(64, 3)
        self.context_head = nn.Linear(64, 16)
        self.actor = nn.Sequential(nn.Linear(64, 512), nn.ELU(),
                                   nn.Linear(512, 256), nn.ELU(),
                                   nn.Linear(256, 128), nn.ELU(), nn.Linear(128, 12))
        for module, prefix in ((self.encoder, "cenet.encoder."),
                               (self.velocity_head, "cenet.velocity_head."),
                               (self.context_head, "cenet.mean_head."),
                               (self.actor, "mlp.")):
            module.load_state_dict({key[len(prefix):]: value.cpu()
                                    for key, value in state.items() if key.startswith(prefix)})

    @torch.jit.export
    def encode(self, packed_history: torch.Tensor):
        normalized = (packed_history - self.history_mean) / (self.history_std + self.eps)
        encoded = self.encoder(normalized)
        return normalized, self.velocity_head(encoded), self.context_head(encoded)

    def forward(self, history: torch.Tensor) -> torch.Tensor:
        if history.dim() != 3 or history.shape[1] != 5 or history.shape[2] != 45:
            raise RuntimeError("DreamWaQ expects (batch, 5, 45)")
        packed = torch.cat((history[:, :, 0:3].reshape(-1, 15),
                            history[:, :, 3:6].reshape(-1, 15),
                            history[:, :, 6:9].reshape(-1, 15),
                            history[:, :, 9:21].reshape(-1, 60),
                            history[:, :, 21:33].reshape(-1, 60),
                            history[:, :, 33:45].reshape(-1, 60)), dim=1)
        _, velocity, context = self.encode(packed)
        observation = torch.cat((history[:, -1, :], velocity, context), dim=1)
        return self.actor((observation - self.mean) / (self.std + self.eps)).clamp(-10.0, 10.0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    checkpoint = torch.load(args.checkpoint, map_location="cpu", weights_only=False)
    assert checkpoint["pion_action_contract"] == dict(
        version=1, reference="current_joint_position", scale=0.25, anchor="policy_step")
    contract = checkpoint["pion_baseline"]
    assert contract["estimator"] == "dreamwaq_cenet19"
    assert contract["actor_dim"] == 64 and contract["history_dim"] == 225
    assert contract["latent_dim"] == 16
    assert contract["actor_order"] == "policy45,velocity3,context16"
    if args.output.exists():
        raise FileExistsError(args.output)
    policy = NativePionPolicy(DreamWaQPolicy(checkpoint["actor_state_dict"])).eval()
    scripted = torch.jit.script(policy)
    with torch.inference_mode():
        result = scripted(torch.zeros(1, 5, 45))
        assert result.shape == (1, 12) and torch.isfinite(result).all()
    torch.jit.save(scripted, str(args.output))
    print(f"Exported complete native policy: {args.output}")


if __name__ == "__main__":
    main()
