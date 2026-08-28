import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    default_urdf = os.path.join(pkg_dir, "urdf", "nxp_jaguar.urdf")
    default_rviz = os.path.join(pkg_dir, "config", "jaguar_display.rviz")

    model_arg = DeclareLaunchArgument(
        "model",
        default_value=default_urdf,
        description="Absolute path to robot URDF file",
    )
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz,
        description="Absolute path to RViz config file",
    )

    with open(default_urdf, "r") as f:
        robot_desc = f.read()

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_desc}],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
    )

    return LaunchDescription([
        model_arg,
        rviz_config_arg,
        rsp_node,
        rviz_node,
    ])
