"""Reconstruct CENet from checkpoint tensors and compare the deployment paths."""

from pathlib import Path
import sys

import pytest
import torch
import torch.nn.functional as F

ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from pion_policy import infer_with_trace

EXPORTS = [
    ("pion_qr1_dreamwaq_clearance_6600_20261123", "pion_qr1_dreamwaq_clearance_native_6600_20261123"),
    ("pion_qr1_dreamwaq_l2c2_height19_9999_20261120", "pion_qr1_dreamwaq_l2c2_height19_native_9999_20261120"),
]

@pytest.mark.parametrize("source_tag,native_tag", EXPORTS)
@pytest.mark.parametrize("batch", [1, 16, 64])
@pytest.mark.parametrize("scale", [0.0, 1.0, 10.0])
def test_complete_dreamwaq_export_matches_checkpoint_and_shipped_actor(batch, scale, source_tag, native_tag):
    checkpoint = torch.load(ROOT / f"models/model_{source_tag}.pt",
                            map_location="cpu", weights_only=False)
    state = checkpoint["actor_state_dict"]
    policy = torch.jit.load(str(ROOT / f"models/policy_{native_tag}.pt")).eval()
    actor = torch.jit.load(str(ROOT / f"models/policy_{source_tag}_raw.pt")).eval()
    history = torch.randn(batch, 5, 45, generator=torch.Generator().manual_seed(42)) * scale
    packed = torch.cat([history[:, :, a:b].flatten(1)
                        for a, b in [(0, 3), (3, 6), (6, 9), (9, 21), (21, 33), (33, 45)]], 1)
    encoded = (packed - state['cenet.history_normalizer._mean']) / (state['cenet.history_normalizer._std'] + 0.01)
    for layer in (0, 2):
        encoded = F.elu(F.linear(encoded, state[f'cenet.encoder.{layer}.weight'],
                                state[f'cenet.encoder.{layer}.bias']))
    velocity = F.linear(encoded, state['cenet.velocity_head.weight'], state['cenet.velocity_head.bias'])
    context = F.linear(encoded, state['cenet.mean_head.weight'], state['cenet.mean_head.bias'])
    observation = torch.cat((history[:, -1], velocity, context), 1)
    expected = (observation - state['obs_normalizer._mean']) / (state['obs_normalizer._std'] + 0.01)
    for layer in (0, 2, 4, 6):
        expected = F.linear(expected, state[f'mlp.{layer}.weight'], state[f'mlp.{layer}.bias'])
        if layer != 6:
            expected = F.elu(expected)
    with torch.inference_mode():
        actual = policy(history)
        traced, trace = infer_with_trace(policy, history)
        torch.testing.assert_close(actual, expected.clamp(-10, 10), rtol=0, atol=0)
        torch.testing.assert_close(actual, actor(observation).clamp(-10, 10), rtol=0, atol=0)
        torch.testing.assert_close(actual, traced, rtol=0, atol=0)
        torch.testing.assert_close(actual, policy(history), rtol=0, atol=0)
        assert len(trace['estimator_context']) == 16 and len(trace['actor_input']) == 64
        assert torch.isfinite(actual).all() and actual.abs().max() <= 10


@pytest.mark.parametrize("source_tag,native_tag", EXPORTS)
def test_shipped_actor_only_export_requires_augmented_observation(source_tag, native_tag):
    policy = torch.jit.load(str(ROOT / f"models/policy_{source_tag}.pt"))
    with pytest.raises((RuntimeError, torch.jit.Error)):
        policy(torch.zeros(1, 5, 45))
