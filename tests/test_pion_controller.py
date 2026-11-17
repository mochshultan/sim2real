# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Protect native Pion inference and named feedback-to-command correspondence."""

from pathlib import Path
import sys

import numpy as np
import rclpy
import torch
from sensor_msgs.msg import JointState

sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))
from pion_controller import PionControllerNode
from pion_policy import JOINT_NAMES, STANDING

ROOT = Path(__file__).parents[1]
NATIVE = ROOT / "models/policy_pion_current_native_9999_20261117.pt"


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


def test_native_ros_controller_anchors_each_policy_step_to_named_measured_feedback():
    rclpy.init(args=["--ros-args", "--params-file", str(ROOT / "config/sim2real_pion.yaml"),
                     "-p", f"policy_path:={NATIVE}", "-p", "policy_reference:=current",
                     "-p", "history_initialization:=repeat", "-p", "record_session:=false"])
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
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
