# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Protect native Pion inference and named feedback-to-command correspondence."""

from pathlib import Path
import sys
import time
from unittest.mock import Mock

import numpy as np
import pytest
import rclpy
import torch
from sensor_msgs.msg import JointState, Joy

sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))
from pion_controller import PionControllerNode
from pion_policy import JOINT_NAMES, STANDING, infer_with_trace

ROOT = Path(__file__).parents[1]
NATIVE = ROOT / "models/policy_pion_current_native_9999_20261117.pt"
CURRENT = ROOT / "models/policy_pion_qr1_dreamwaq_l2c2_height19_native_9999_20261120.pt"
PREVIOUS = ROOT / "models/policy_pion_qr1_plane_orient_9999_20261120b.pt"


@pytest.mark.parametrize("policy_path,expected_dim", [(CURRENT, 48), (PREVIOUS, 64)])
def test_controller_rejects_model_from_other_actor_dimension(policy_path, expected_dim):
    rclpy.init(args=["--ros-args", "--params-file", str(ROOT / "config/sim2real_pion.yaml"),
                     "-p", f"policy_path:={policy_path}", "-p", "policy_reference:=current",
                     "-p", "history_initialization:=repeat", "-p", "record_session:=false",
                     "-p", f"actor_input_dim:={expected_dim}"])
    try:
        with pytest.raises(ValueError, match="Actor dimension mismatch"):
            PionControllerNode()
    finally:
        rclpy.shutdown()


def test_dreamwaq_recording_path_matches_direct_policy_and_describes_context(tmp_path):
    import json
    rclpy.init(args=["--ros-args", "--params-file", str(ROOT / "config/sim2real_pion.yaml"),
                     "-p", f"policy_path:={CURRENT}", "-p", "policy_reference:=current",
                     "-p", "history_initialization:=repeat", "-p", f"session_log_dir:={tmp_path}",
                     "-p", "record_session:=true"])
    node = None
    try:
        node = PionControllerNode()
        history = torch.randn(1, 5, 45, generator=torch.Generator().manual_seed(12))
        node._session_sample = {}
        with torch.inference_mode():
            torch.testing.assert_close(node._infer_temporal_policy(history), node.policy(history), rtol=0, atol=0)
        assert len(node._session_sample['actor_input']) == 64
        assert len(node._session_sample['estimator_context']) == 16
        path = node.session_recorder.path
        assert node.session_recorder.close()
        metadata = json.loads(path.read_text().splitlines()[0])
        assert '64D DreamWaQ' in metadata['inference_trace_fields']['actor_input']
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize("policy_path", [
    CURRENT,
    PREVIOUS,
    NATIVE, ROOT / "models/policy_pion_current_height20_swing_native_14999_20261118.pt",
])
def test_recorded_inference_matches_export_for_varied_histories(policy_path):
    policy = torch.jit.load(str(policy_path)).eval()
    generator = torch.Generator().manual_seed(39)
    with torch.no_grad():
        for scale in (0.0, 0.1, 1.0, 10.0):
            history = torch.randn(1, 5, 45, generator=generator) * scale
            action, trace = infer_with_trace(policy, history)
            torch.testing.assert_close(action, policy(history), rtol=0, atol=0)
            assert len(trace['estimator_input']) == 225
            assert len(trace['estimator_output']) == 3
            assert len(trace['actor_input']) == policy.policy.mean.numel()
            if policy_path == CURRENT:
                assert len(trace['estimator_context']) == 16
                assert len(trace['actor_input']) == 64
            assert len(trace['actor_output']) == 12


def test_native_export_preserves_the_existing_verified_actor_for_named_joint_histories():
    native = torch.jit.load(str(NATIVE))
    legacy = torch.jit.load(str(ROOT / "models/policy_pion_current_baseline_9999_20261117.pt"))
    # Independently name every physical joint in the legacy API.
    legacy_names = [f"{leg}_{axis}_joint" for axis in ("collar", "hip", "knee")
                    for leg in ("FR", "FL", "BR", "BL")]
    source_indices = [JOINT_NAMES.index(name) for name in legacy_names]
    obs_indices = list(range(9)) + [offset + index for offset in (9, 21, 33) for index in source_indices]
    history = torch.randn(16, 5, 45, generator=torch.Generator().manual_seed(42))
    with torch.inference_mode():
        native_action = native(history)
        legacy_action = legacy(history[:, :, obs_indices])
    for legacy_index, name in enumerate(legacy_names):
        torch.testing.assert_close(legacy_action[:, legacy_index], native_action[:, JOINT_NAMES.index(name)], rtol=0, atol=0)
    assert native.action_reference == "current"
    assert native.clip_actions == 10.0


@pytest.mark.parametrize("policy_path", [
    CURRENT,
    PREVIOUS,
    NATIVE,
    ROOT / "models/policy_pion_current_height20_swing_native_14999_20261118.pt",
])
def test_native_ros_controller_anchors_each_policy_step_to_named_measured_feedback(policy_path):
    expected_dim = torch.jit.load(str(policy_path)).policy.mean.numel()
    rclpy.init(args=["--ros-args", "--params-file", str(ROOT / "config/sim2real_pion.yaml"),
                     "-p", f"policy_path:={policy_path}", "-p", "policy_reference:=current",
                     "-p", "history_initialization:=repeat", "-p", "record_session:=false",
                     "-p", f"actor_input_dim:={expected_dim}"])
    node = None
    try:
        node = PionControllerNode()
        packet_names = [f"{leg}_{axis}_joint" for leg in ("Bl", "Br", "Fl", "Fr")
                        for axis in ("roll", "hip_pitch", "knee")]
        native_positions = STANDING + np.linspace(-0.015, 0.015, 12, dtype=np.float32)
        packet_to_native = [node.joint_name_to_index[name] for name in packet_names]
        packet = JointState(name=packet_names, position=native_positions[packet_to_native].tolist(),
                            velocity=np.linspace(-0.1, 0.1, 12).tolist(), effort=[0.0] * 12)
        node._joint_state_cb(packet)
        np.testing.assert_array_equal(node.joint_pos, native_positions)
        assert node.joints_received
        node.body_quat[:] = [0, 0, 0, 1]
        initial = node.obs_builder.build_step_observation(np.zeros(3), node.body_quat,
                                                         np.zeros(3), node.joint_pos, node.joint_vel)
        node.obs_builder.reset_history(initial)
        np.testing.assert_array_equal(node.obs_builder.history_buf[0, :, 9:21], np.tile(native_positions, (5, 1)))
        # Exercise two anchors, including the first WALK step where the old ramp suppressed actions.
        for shift in (0.0, 0.01):
            measured = native_positions + shift
            target, velocity, kp, kd = node._walk_targets(node.walk_start_time, measured,
                                                         node.joint_vel, node.body_quat,
                                                         np.zeros(3), np.zeros(3))
            action = node.obs_builder.last_action
            np.testing.assert_allclose(target, measured + 0.25 * action, rtol=0, atol=2e-7)
            assert max(abs(action)) <= 10
            np.testing.assert_array_equal(velocity, np.zeros(12))
            assert kp == [45.0] * 12 and kd == [4.5] * 12
        # Return from WALK on tilted ground: leveling belongs to stand states,
        # with left/right signs in native Pion order, not the legacy Jaguar order.
        node.state = "WALK"
        node.imu_received = True
        node.last_imu_time = time.monotonic()
        node.last_joints_time = time.monotonic()
        node.body_quat[:] = [np.sin(0.1), 0, 0, np.cos(0.1)]
        node._joy_cb(Joy(buttons=[0] * 7 + [1] + [0] * 3))
        assert node.state == "STANDUP"
        trim = node._compute_imu_stabilization(1.0, node.state)
        assert trim[0] < 0 and trim[1] > 0 and trim[2] < 0 and trim[3] > 0
        assert np.max(np.abs(trim[:4])) <= 0.12
        np.testing.assert_array_equal(trim[8:], np.zeros(4))
        np.testing.assert_array_equal(node._compute_imu_stabilization(1.0, "STAND_HOLD"), trim)
        np.testing.assert_array_equal(node._compute_imu_stabilization(1.0, "WALK"), np.zeros(12))
        node.body_quat[:] = [0, np.sin(0.1), 0, np.cos(0.1)]
        pitch_trim = node._compute_imu_stabilization(1.0, "STAND_HOLD")
        assert np.all(pitch_trim[4:8] < 0) and np.max(np.abs(pitch_trim[4:8])) <= 0.15
        np.testing.assert_array_equal(pitch_trim[8:], np.zeros(4))
        node.last_imu_time = time.monotonic() - 1.0
        np.testing.assert_array_equal(node._compute_imu_stabilization(1.0, "STAND_HOLD"), np.zeros(12))
        # A Pion command must already obey the final driver guard when published.
        node.joint_cmd_pub = Mock()
        node.safe_park_active_pub = Mock()
        node.state = "WALK"
        node.last_imu_time = node.last_joints_time = time.monotonic()
        node.joint_pos[4:8] = 0.8
        assert node._sensors_ready()  # The new positive hip range must not trip the old +0.2 guard.
        node.joint_pos[:] = native_positions
        requested = np.array([-1., 1., -1., 1., -4., 1., -4., 1., -1., 4., -1., 4.])
        assert node._publish_targets(requested, np.zeros(12), [45.] * 12, [4.5] * 12,
                                     node.get_clock().now().nanoseconds / 1e9)
        published = node.joint_cmd_pub.publish.call_args.args[0]
        np.testing.assert_allclose(published.position,
                                  [-0.7, 0.7, -0.7, 0.7, -2.0, 1.0, -2.0, 1.0, -0.05, 1.6, -0.05, 1.6])
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
