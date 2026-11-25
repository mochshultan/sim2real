# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Pion QR1 DreamWaQ VHIP mean-context checkpoint9999: native history and current-position actions."""

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
                default_value=str(package / "models/history/dreamwaq_vhip_mean_10k/policy_pion_qr1_dreamwaq_vhip_mean_native_9999_20261125.pt"),
                description="Pion QR1 DreamWaQ VHIP mean-context checkpoint9999 with complete CENet inference",
            ),
            DeclareLaunchArgument("policy_reference", default_value="current", choices=["current"]),
            DeclareLaunchArgument("history_initialization", default_value="repeat", choices=["repeat"]),
            DeclareLaunchArgument("actor_input_dim", default_value="64", choices=["64"]),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(package / "launch/sim2real.launch.py"))),
        ]
    )
