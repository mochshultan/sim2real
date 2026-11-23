#!/usr/bin/env python3
# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Native Pion Current controller with a verified checkpoint deployment contract.

The existing hardware state machine and stop guards are shared; policy state,
feedback indexing, observation history and actions stay in native Pion order.
"""

import hashlib
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy

from nxp_jaguar_controller import NXPJaguarControllerNode, POSITION_LIMIT_TOLERANCE
from pion_policy import ACTION_SCALE, CLIP_ACTIONS, CONTROL_DT, JOINT_NAMES, KD, KP
from pion_policy import NAME_TO_PION, ROS_TO_PION, STANDING, PionObservationBuilder, infer_with_trace


class PionControllerNode(NXPJaguarControllerNode):
    controller_node_name = "pion_controller"
    estimator_target_scale = 2.0
    joint_names = JOINT_NAMES
    joint_name_to_index = NAME_TO_PION
    ros_to_policy = ROS_TO_PION
    roll_trim_signs = (-1.0, 1.0, -1.0, 1.0)  # FL, FR, BL, BR
    joint_feedback_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    # Final calibrated-coordinate target guard: intersection of the controller
    # limits and the existing CAN-driver mechanical limits, in native Pion order.
    joint_target_lower = np.array([-0.7] * 4 + [-2.0] * 4 + [-0.05] * 4, dtype=np.float32)
    joint_target_upper = np.array([0.7] * 4 + [1.0] * 4 + [1.6] * 4, dtype=np.float32)

    joint_feedback_lower = joint_target_lower - POSITION_LIMIT_TOLERANCE
    joint_feedback_upper = joint_target_upper + POSITION_LIMIT_TOLERANCE

    def _configure_parameters(self):
        self.declare_parameter("actor_input_dim", 0)
        self.expected_actor_dim = int(self.get_parameter("actor_input_dim").value)
        if self.expected_actor_dim not in (0, 48, 64):
            raise ValueError("actor_input_dim must be 0, 48 or 64")
        policy_path = super()._configure_parameters()
        if self.policy_reference != "current" or self.history_initialization != "repeat":
            raise ValueError("Pion Current requires current reference and repeat history initialization")
        if self.action_ema_alpha != 0 or self.walk_ramp_duration != 0:
            raise ValueError("Pion WALK training parity requires no action EMA and no WALK ramp")
        if not (np.allclose(self.rl_kp, KP) and np.allclose(self.rl_kd, KD)):
            raise ValueError("This Pion checkpoint requires Kp=45 and Kd=4.5")
        self.stand_joint_pos = STANDING.copy()
        return policy_path

    def _infer_temporal_policy(self, history):
        if self._session_sample is None:
            return self.policy(history)
        started = time.time_ns()
        action, trace = infer_with_trace(self.policy, history)
        self._session_sample.update(trace)
        self._session_sample["policy_input"] = history[0].tolist()
        self._session_sample["inference_start_wall_time_ns"] = started
        self._session_sample["inference_end_wall_time_ns"] = time.time_ns()
        return action

    def _new_observation_builder(self):
        return PionObservationBuilder()

    def _load_policy(self, policy_path):
        super()._load_policy(policy_path)
        if getattr(self.policy, "joint_order", "") != "pion_native_FL_FR_BL_BR":
            raise ValueError("Pion controller requires the native Pion export; Jaguar-order exports are incompatible")
        if getattr(self.policy, "clip_actions", None) != CLIP_ACTIONS:
            raise ValueError("Pion export must clip actions to [-10,10], matching training")
        actual_actor_dim = self.policy.policy.mean.numel()
        if self.expected_actor_dim and actual_actor_dim != self.expected_actor_dim:
            raise ValueError(f"Actor dimension mismatch: launch expects {self.expected_actor_dim}D, policy uses {actual_actor_dim}D")
        self.get_logger().info(
            f"Pion contract: joints={','.join(JOINT_NAMES)}; dt={CONTROL_DT}; "
            f"q_target=q_measured+{ACTION_SCALE}*action; Kp={KP}; Kd={KD}; "
            "WALK ramp=off; EMA=off; observation reference=0; history=repeat"
        )
        policy_file = Path(policy_path)
        if not policy_file.name.startswith("policy_"):
            raise ValueError("Pion export must have a versioned policy filename and paired deployment manifest")
        manifest_file = policy_file.with_name("deployment_" + policy_file.stem[len("policy_"):] + ".json")
        manifest = json.loads(manifest_file.read_text())
        if manifest["policy_file"] != policy_file.name:
            raise ValueError("Pion deployment manifest names a different policy file")
        actual_hash = hashlib.sha256(policy_file.read_bytes()).hexdigest()
        if actual_hash != manifest["policy_sha256"]:
            raise ValueError("Policy hash differs from the paired Pion deployment manifest")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PionControllerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
