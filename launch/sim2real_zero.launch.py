# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Launch Jaguar AdaBoot L2C2 checkpoint 5999 with a zero joint reference."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    """Reuse the hardware stack with the matching zero-reference policy."""
    package_dir = Path(__file__).resolve().parents[1]
    policy_path = package_dir / "models/policy_adaboot_ideal_l2c2_zero_reference_5999_20261103.pt"
    return LaunchDescription([
        DeclareLaunchArgument(
            "policy_path",
            default_value=str(policy_path),
            description="TorchScript policy trained with a zero joint reference (default: Jaguar 5999)",
        ),
        DeclareLaunchArgument(
            "policy_reference",
            default_value="zero",
            choices=["zero"],
            description="Zero joint reference for observations and action targets",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(package_dir / "launch/sim2real.launch.py")),
        )
    ])
