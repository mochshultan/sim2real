"""Verify the deployed policy's observation order, scaling and history contract."""

from pathlib import Path
import sys

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))
from policy_observation import JaguarObservationBuilder


def test_observation_matches_training_scales_and_joint_reference():
    reference = np.linspace(-1.5, 1.2, 12, dtype=np.float32)
    builder = JaguarObservationBuilder(reference)
    action = np.linspace(-0.5, 0.5, 12, dtype=np.float32)
    builder.update_last_action(action)
    obs = builder.build_step_observation(
        np.array([4.0, -8.0, 2.0]), [0, 0, 0, 1],
        np.array([0.3, -0.2, 0.4]), reference + 0.1, np.full(12, 2.0),
    )
    expected = np.concatenate((
        [1.0, -2.0, 0.5], [0.0, 0.0, -1.0], [0.6, -0.4, 0.1],
        np.full(12, 0.1), np.full(12, 0.1), action,
    ))
    assert obs.shape == (45,)
    assert obs.dtype == np.float32
    np.testing.assert_allclose(obs, expected, atol=1e-7)


def test_history_is_time_major_and_reset_clears_previous_actions():
    builder = JaguarObservationBuilder(np.zeros(12))
    for frame in range(1, 7):
        history = builder.update_and_get_history(np.full(45, frame))
    assert tuple(history.shape) == (1, 5, 45)
    np.testing.assert_array_equal(history[0, :, 0], [2, 3, 4, 5, 6])
    builder.update_last_action(np.ones(12))
    builder.reset_history(np.full(45, 9))
    np.testing.assert_array_equal(builder.history_buf[0, :-1], 0)
    np.testing.assert_array_equal(builder.history_buf[0, -1], 9)
    np.testing.assert_array_equal(builder.last_action, 0)


def test_deployed_policy_accepts_extracted_observation_builder():
    root = Path(__file__).parents[1]
    reference = np.array([0.0] * 4 + [-1.51, -1.51, -1.49, -1.49]
                         + [1.22, 1.22, 1.20, 1.20], dtype=np.float32)
    builder = JaguarObservationBuilder(reference)
    obs = builder.build_step_observation(
        np.zeros(3), [0, 0, 0, 1], np.zeros(3), reference, np.zeros(12),
    )
    policy = torch.jit.load(str(root / "models/policy.pt"), map_location="cpu").eval()
    with torch.no_grad():
        result = policy(builder.update_and_get_history(obs))
    assert tuple(result.shape) == (1, 12)
    assert torch.isfinite(result).all()



def test_repeat_history_cold_start_clears_previous_action_feedback():
    builder = JaguarObservationBuilder(np.zeros(12), history_initialization="repeat")
    builder.last_action[:] = 8.0
    initial = np.arange(45, dtype=np.float32) / 10.0
    builder.reset_history(initial)
    expected = initial.copy()
    expected[33:45] = 0.0
    np.testing.assert_allclose(builder.history_buf, np.tile(expected, (1, 5, 1)))
    np.testing.assert_array_equal(builder.last_action, np.zeros(12))
    history = builder.update_and_get_history(np.ones(45, dtype=np.float32)).numpy()
    np.testing.assert_allclose(history[0, :4], np.tile(expected, (4, 1)))
    np.testing.assert_allclose(history[0, 4], 1.0)
