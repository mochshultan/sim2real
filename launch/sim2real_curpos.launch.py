# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Pion Height20 checkpoint14999: measured-position anchor, zero observation reference."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package = Path(__file__).resolve().parents[1]
    return LaunchDescription(
        [
            DeclareLaunchArgument("session_log_dir", default_value=str(package / "data/sessions/curpos")),
            DeclareLaunchArgument("config_file", default_value=str(package / "config/sim2real_pion.yaml")),
            DeclareLaunchArgument("controller_executable", default_value="pion_controller.py", choices=["pion_controller.py"]),
            DeclareLaunchArgument("controller_name", default_value="pion_controller", choices=["pion_controller"]),
            DeclareLaunchArgument(
                "policy_path",
                default_value=str(package / "models/policy_pion_current_height20_swing_native_14999_20261118.pt"),
                description="Pion Height20 Swing15k TorchScript, native Pion training joint order",
            ),
            DeclareLaunchArgument("policy_reference", default_value="current", choices=["current"]),
            DeclareLaunchArgument("history_initialization", default_value="repeat", choices=["repeat"]),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(package / "launch/sim2real.launch.py"))),
        ]
    )
