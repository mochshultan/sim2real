# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Pion Current checkpoint9999: measured-position action anchor, zero observation reference."""

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
                default_value=str(package / "models/policy_pion_current_baseline_9999_20261117.pt"),
                description="Pion Current10k TorchScript, controller-order adapter included",
            ),
            DeclareLaunchArgument("policy_reference", default_value="current", choices=["current"]),
            DeclareLaunchArgument("history_initialization", default_value="repeat", choices=["repeat"]),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(package / "launch/sim2real.launch.py"))),
        ]
    )
