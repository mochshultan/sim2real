"""Run only the Jaguar CAN node with the copied CyberGear RS00 protocol."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():
    share = Path(get_package_share_directory("jaguar_control"))
    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=str(share / "config" / "sim2real.yaml")),
        DeclareLaunchArgument("motor_params_file", default_value=str(share / "config" / "cybergear_motor_params.yaml")),
        Node(
            package="jaguar_control",
            executable="robstride_can_node",
            name="robstride_can_hardware",
            output="screen",
            parameters=[
                LaunchConfiguration("config_file"),
                {"motor_params_file": LaunchConfiguration("motor_params_file"), "startup_clear_faults": False},
            ],
        ),
    ])
