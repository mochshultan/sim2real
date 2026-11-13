# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Launch Pion AdaBoot-L2C2 checkpoint 11500 with zero reference and embedded joint mapping."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    """Reuse the hardware stack with the matching zero-reference policy."""
    package_dir = Path(__file__).resolve().parents[1]
    policy_path = package_dir / "models/policy_pion_adaboot_l2c2_zero_32v_height25_11500_20261113.pt"
    return LaunchDescription([
        DeclareLaunchArgument(
            "policy_path",
            default_value=str(policy_path),
            description="Zero-reference TorchScript policy (default: Pion 11500, controller-order adapter included)",
        ),
        DeclareLaunchArgument(
            "policy_reference",
            default_value="zero",
            choices=["zero"],
            description="Zero joint reference for observations and action targets",
        ),
        DeclareLaunchArgument(
            "history_initialization", default_value="repeat", choices=["zeros", "repeat"],
            description="Repeat the first observation as in model 11500 training",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(package_dir / "launch/sim2real.launch.py")),
        )
    ])
