"""Protect Pion PACE's hardware command and recorded-data contracts."""

import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]


def load_tool(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / "pace" / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


chirp = load_tool("run_pion_chirp")
converter = load_tool("convert_to_pace")


def test_command_matches_driver_mit_layout_and_pace_zero_velocity():
    """A target slope must not become velocity feed-forward during PACE replay."""
    node = chirp.PionChirp.__new__(chirp.PionChirp)
    sent = []
    node.JointState = lambda: SimpleNamespace(header=SimpleNamespace())
    node.node = SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: "stamp")))
    node.command_pub = SimpleNamespace(publish=sent.append)
    target = list(chirp.NOMINAL_STAND_POSITION)
    node.publish_command(target, [0.1] * 12, hold_all_joints=True)
    assert sent[0].name == list(chirp.JOINTS)
    assert sent[0].position == target
    assert sent[0].velocity == [0.0] * 12
    assert sent[0].effort == [45.0] * 12 + [4.5] * 12 + [0.0] * 12
    node.publish_command(target, [0.0] * 12, gain_scale=0.0)
    assert sent[-1].effort == [0.0] * 36


def test_stale_feedback_stops_before_more_commands():
    node = chirp.PionChirp.__new__(chirp.PionChirp)
    node.positions = list(chirp.NOMINAL_STAND_POSITION)
    node.velocities = [0.0] * 12
    node.torques = [0.0] * 12
    node.feedback_time = -1.0
    with pytest.raises(RuntimeError, match="feedback timed out"):
        node.check_feedback(node.positions)


def test_named_hardware_data_maps_to_pion_policy_order():
    # Values encode the physical hardware index, independently of PACE ordering.
    hardware_names = list(chirp.JOINTS)
    expected = [6, 9, 0, 3, 7, 10, 1, 4, 8, 11, 2, 5]
    ordered = converter.reorder_joint_values(hardware_names, list(range(12)), "state")
    np.testing.assert_array_equal(ordered, expected)
    alias_names = [
        n.replace("FR", "Fr")
        .replace("FL", "Fl")
        .replace("BR", "Br")
        .replace("BL", "Bl")
        .replace("collar", "roll")
        .replace("_hip_", "_hip_pitch_")
        for n in hardware_names
    ]
    np.testing.assert_array_equal(converter.reorder_joint_values(alias_names, list(range(12)), "state"), expected)
    with pytest.raises(ValueError, match="duplicate"):
        converter.reorder_joint_values([hardware_names[0]] * 12, [0.0] * 12, "state")


def test_converter_preserves_command_steps_at_400hz_and_rejects_missing_intervals():
    state_time = np.arange(0.0, 1.001, 0.005)
    measured = np.repeat(state_time[:, None], 12, axis=1)
    command_time = np.arange(0.0, 1.001, 0.02)
    targets = np.repeat((command_time >= 0.5)[:, None], 12, axis=1).astype(float)
    t, q, desired = converter.align_trajectory(state_time, measured, command_time, targets, 400, 0.02, 0.06)
    np.testing.assert_allclose(np.diff(t), 0.0025, atol=1e-7)
    np.testing.assert_allclose(q[:, 0], t, atol=1e-7)
    np.testing.assert_array_equal(desired[:, 0], t >= 0.5)
    valid = (state_time < 0.4) | (state_time > 0.6)
    with pytest.raises(ValueError, match="Missing/stale"):
        converter.align_trajectory(state_time[valid], measured[valid], command_time, targets, 400, 0.02, 0.06)


def test_fit_and_validation_excitation_respect_pion_joint_bounds():
    for profile in chirp.PROFILES.values():
        for phase_index, hardware_index in enumerate(chirp.ACTIVE_JOINT_INDICES):
            nominal = chirp.NOMINAL_STAND_POSITION[hardware_index]
            targets = np.array(
                [chirp.chirp_target(t, phase_index, nominal, 0.03, profile) for t in np.arange(0, 20, 0.02)]
            )
            low, high = chirp.JOINT_LIMITS[hardware_index]
            assert np.min(targets) > low + 0.1
            assert np.max(targets) < high - 0.1
            assert np.max(np.abs(np.diff(targets))) * 50 < 0.15


def test_epoch_timestamps_keep_the_replay_physics_grid_uniform():
    epoch = 1_800_000_000.0
    states = epoch + np.arange(0, 1.001, 0.005)
    commands = epoch + np.arange(0, 1.001, 0.02)
    q = np.zeros((len(states), 12))
    targets = np.zeros((len(commands), 12))
    t, _, _ = converter.align_trajectory(states, q, commands, targets, 400, 0.02, 0.06, epoch + 0.1, epoch + 0.9)
    np.testing.assert_allclose(np.diff(t), 0.0025, atol=1e-7)


def test_duplicate_chirp_publishers_cannot_bypass_exclusive_control_by_name():
    node = chirp.PionChirp.__new__(chirp.PionChirp)

    def publishers(topic):
        return [SimpleNamespace(node_name="pion_pace_chirp")] * 2 if topic == "/joint_commands" else []

    node.node = SimpleNamespace(get_publishers_info_by_topic=publishers, get_name=lambda: "pion_pace_chirp")
    with pytest.raises(RuntimeError, match="Competing publisher"):
        node.ensure_no_competitors()
