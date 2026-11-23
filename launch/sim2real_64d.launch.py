"""Current-position deployment with the 64D DreamWaQ CENet actor."""

from pathlib import Path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package = Path(__file__).resolve().parents[1]
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(package / "launch/sim2real_curpos.launch.py"))),
    ])
