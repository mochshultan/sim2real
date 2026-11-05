"""Controller refactoring must preserve trajectories, policy blending and guards."""

from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
import torch

sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))
from nxp_jaguar_controller import (
    NXPJaguarControllerNode as Controller, DEFAULT_JOINT_POS, STAND_JOINT_POS,
)
from policy_observation import JaguarObservationBuilder


@pytest.mark.parametrize("duration", [2.0, 4.0])
@pytest.mark.parametrize("fraction,blend", [(0.0, 0.0), (0.5, 0.5), (1.0, 1.0)])
def test_shared_transition_preserves_cosine_trajectory(duration, fraction, blend):
    node = SimpleNamespace(transition_start_time=10.0,
                           transition_start_pos=np.zeros(12),
                           transition_target_pos=np.ones(12))
    elapsed, alpha, position, velocity = Controller._transition_targets(
        node, 10.0 + duration * fraction, duration,
    )
    assert elapsed == duration * fraction
    assert alpha == fraction
    np.testing.assert_allclose(position, blend, atol=1e-7)
    expected_velocity = np.pi / (2 * duration) if fraction == 0.5 else 0.0
    np.testing.assert_allclose(velocity, expected_velocity, atol=1e-7)


def walk_node(action):
    return SimpleNamespace(
        obs_builder=JaguarObservationBuilder(DEFAULT_JOINT_POS),
        policy=lambda history: torch.tensor([action], dtype=torch.float32),
        policy_expects_temporal=True, action_ema_alpha=0.0,
        filtered_action=np.zeros(12), walk_start_time=10.0, walk_ramp_duration=0.8,
        rl_kp=[45.0] * 12, rl_kd=[4.5] * 12, transition_kd=[4.5] * 12,
        debug_pub=Mock(), _trigger_safe_shutdown=Mock(),
    )


@pytest.mark.parametrize("elapsed,ramp", [(0.0, 0.0), (0.4, 0.5), (0.8, 1.0)])
def test_walk_targets_preserve_policy_blend_and_gains(elapsed, ramp):
    node = walk_node([0.2] * 12)
    targets = Controller._walk_targets(node, 10.0 + elapsed, DEFAULT_JOINT_POS,
                                       np.zeros(12), [0, 0, 0, 1], np.zeros(3), np.zeros(3))
    position, velocity, kp, kd = targets
    expected = (1 - ramp) * STAND_JOINT_POS + ramp * DEFAULT_JOINT_POS + 0.25 * ramp * 0.2
    np.testing.assert_allclose(position, expected, atol=1e-7)
    np.testing.assert_array_equal(velocity, 0)
    assert kp == [45.0] * 12
    assert kd == [4.5] * 12


def test_invalid_walk_output_requests_shutdown_and_returns_no_target():
    node = walk_node([float("nan")] * 12)
    result = Controller._walk_targets(node, 11.0, DEFAULT_JOINT_POS,
                                      np.zeros(12), [0, 0, 0, 1], np.zeros(3), np.zeros(3))
    assert result is None
    node._trigger_safe_shutdown.assert_called_once_with(11.0, "Invalid policy output")
    node.debug_pub.publish.assert_not_called()


def test_tracking_fault_blocks_publication_after_extraction():
    node = SimpleNamespace(state="WALK", joint_pos=np.full(12, 5.0),
                           _persistent_fault=Mock(return_value=True),
                           _trigger_safe_shutdown=Mock(), joint_cmd_pub=Mock())
    assert not Controller._publish_targets(node, np.zeros(12), np.zeros(12),
                                           [45.0] * 12, [4.5] * 12, 10.0)
    node._trigger_safe_shutdown.assert_called_once()
    node.joint_cmd_pub.publish.assert_not_called()
