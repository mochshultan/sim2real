#!/usr/bin/env python3
# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Native Pion Current controller with the model_9999 training contract.

The existing hardware state machine and stop guards are shared; policy state,
feedback indexing, observation history and actions stay in native Pion order.
"""

import hashlib
import json
from pathlib import Path

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy

from nxp_jaguar_controller import NXPJaguarControllerNode
from pion_policy import ACTION_SCALE, CLIP_ACTIONS, CONTROL_DT, JOINT_NAMES, KD, KP
from pion_policy import NAME_TO_PION, ROS_TO_PION, STANDING, PionObservationBuilder
from project_paths import package_file


class PionControllerNode(NXPJaguarControllerNode):
    controller_node_name = "pion_controller"
    joint_names = JOINT_NAMES
    joint_name_to_index = NAME_TO_PION
    ros_to_policy = ROS_TO_PION
    joint_feedback_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

    def _configure_parameters(self):
        policy_path = super()._configure_parameters()
        if self.policy_reference != "current" or self.history_initialization != "repeat":
            raise ValueError("Pion Current requires current reference and repeat history initialization")
        if self.action_ema_alpha != 0 or self.walk_ramp_duration != 0 or self.use_imu_stabilization:
            raise ValueError("Pion training parity requires no action EMA, no WALK ramp and no standing IMU trim")
        if not (np.allclose(self.rl_kp, KP) and np.allclose(self.rl_kd, KD)):
            raise ValueError("This Pion checkpoint requires Kp=45 and Kd=4.5")
        self.stand_joint_pos = STANDING.copy()
        return policy_path

    def _new_observation_builder(self):
        return PionObservationBuilder()

    def _load_policy(self, policy_path):
        super()._load_policy(policy_path)
        if getattr(self.policy, "joint_order", "") != "pion_native_FL_FR_BL_BR":
            raise ValueError("Pion controller requires the native Pion export; Jaguar-order exports are incompatible")
        if getattr(self.policy, "clip_actions", None) != CLIP_ACTIONS:
            raise ValueError("Pion export must clip actions to [-10,10], matching training")
        self.get_logger().info(
            f"Pion contract: joints={','.join(JOINT_NAMES)}; dt={CONTROL_DT}; "
            f"q_target=q_measured+{ACTION_SCALE}*action; Kp={KP}; Kd={KD}; "
            "WALK ramp=off; EMA=off; observation reference=0; history=repeat"
        )
        manifest = json.loads(package_file("models", "deployment_pion_current_native_9999_20261117.json").read_text())
        actual_hash = hashlib.sha256(Path(policy_path).read_bytes()).hexdigest()
        if actual_hash != manifest["policy_sha256"]:
            raise ValueError("Policy hash differs from the audited Pion model_9999 deployment manifest")


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
