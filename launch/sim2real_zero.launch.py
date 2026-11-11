# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Launch Jaguar AdaBoot L2C2 clearance checkpoint 19999 with a zero joint reference."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    """Reuse the hardware stack with the matching zero-reference policy."""
    package_dir = Path(__file__).resolve().parents[1]
    policy_path = package_dir / "models/policy_adaboot_l2c2_zero_reference_clearance5cm_19999_20261111.pt"
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(package_dir / "launch/sim2real.launch.py")),
            launch_arguments={
                "policy_path": str(policy_path),
                "policy_reference": "zero",
                "history_initialization": "repeat",
            }.items(),
        )
    ])
