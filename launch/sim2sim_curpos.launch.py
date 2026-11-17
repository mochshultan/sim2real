# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Run the real current-position controller against a Pion MuJoCo feedback bridge."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package = Path(__file__).resolve().parents[1]
    arguments = [
        DeclareLaunchArgument("domain_id", default_value="71", description="Isolated simulation DDS domain"),
        DeclareLaunchArgument("viz", default_value="viser", choices=["viser", "none"]),
        DeclareLaunchArgument("port", default_value="8080"),
        DeclareLaunchArgument("duration", default_value="0", description="Wall seconds;0 runs until stopped"),
        DeclareLaunchArgument("auto_walk", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("command_x", default_value="0.0"),
        DeclareLaunchArgument("command_y", default_value="0.0"),
        DeclareLaunchArgument("command_yaw", default_value="0.0"),
        DeclareLaunchArgument("report", default_value="", description="Optional simulation summary JSON"),
    ]
    bridge = Node(
        package="jaguar_control",
        executable="sim2sim_curpos_bridge.py",
        output="screen",
        parameters=[
            {
                "model_path": str(package / "sim2sim/pion_current/pion_qr1.xml"),
                "viz": LaunchConfiguration("viz"),
                "port": ParameterValue(LaunchConfiguration("port"), value_type=int),
                "duration": ParameterValue(LaunchConfiguration("duration"), value_type=float),
                "auto_walk": ParameterValue(LaunchConfiguration("auto_walk"), value_type=bool),
                "command_x": ParameterValue(LaunchConfiguration("command_x"), value_type=float),
                "command_y": ParameterValue(LaunchConfiguration("command_y"), value_type=float),
                "command_yaw": ParameterValue(LaunchConfiguration("command_yaw"), value_type=float),
                "config_file": LaunchConfiguration("config_file"),
                "report": LaunchConfiguration("report"),
            }
        ],
    )
    return LaunchDescription(
        arguments
        + [
            SetEnvironmentVariable("JAGUAR_SIMULATION", "1"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(package / "launch/sim2real_curpos.launch.py")),
                launch_arguments={
                    "domain_id": LaunchConfiguration("domain_id"),
                    "with_hardware": "false",
                    "with_imu": "false",
                    "with_joy": "false",
                    "with_teleop": "false",
                }.items(),
            ),
            bridge,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=bridge,
                    on_exit=[EmitEvent(event=Shutdown(reason="MuJoCo simulation ended"))],
                )
            ),
        ]
    )
