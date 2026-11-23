"""Current-position deployment with the 48D velocity-estimator actor."""

from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package = Path(__file__).resolve().parents[1]
    return LaunchDescription([
        DeclareLaunchArgument("actor_input_dim", default_value="48", choices=["48"]),
        DeclareLaunchArgument("policy_reference", default_value="current", choices=["current"]),
        DeclareLaunchArgument("history_initialization", default_value="repeat", choices=["repeat"]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(package / "launch/sim2real.launch.py"))),
    ])
