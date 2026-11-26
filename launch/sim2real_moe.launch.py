# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Pion measured-URDF MoE-Loco deployment with complete CENet inference."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package = Path(__file__).resolve().parents[1]
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "policy_path",
                default_value=str(
                    package
                    / "models/history/moeloco_measured_l2c2_5k/policy_pion_qr1_moeloco_measured_native_4999_20261126.pt"
                ),
                description="Pion MoE-Loco L2C2 checkpoint4999, four experts and CENet19 mean context",
            ),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(package / "launch/sim2real_curpos.launch.py"))),
        ]
    )
