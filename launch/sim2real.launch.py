import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    default_policy = os.path.join(pkg_dir, "models", "policy.pt")

    # Launch arguments
    policy_path_arg = DeclareLaunchArgument(
        "policy_path",
        default_value=default_policy,
        description="Path to TorchScript policy.pt",
    )
    with_imu_arg = DeclareLaunchArgument(
        "with_imu",
        default_value="true",
        description="Launch serial_imu node",
    )
    with_joy_arg = DeclareLaunchArgument(
        "with_joy",
        default_value="true",
        description="Launch joy_node",
    )
    with_hardware_arg = DeclareLaunchArgument(
        "with_hardware",
        default_value="true",
        description="Launch RS00 CAN hardware driver node",
    )
    with_controller_arg = DeclareLaunchArgument(
        "with_controller",
        default_value="true",
        description="Launch NXP Jaguar RL controller node",
    )

    policy_path = LaunchConfiguration("policy_path")
    with_imu = LaunchConfiguration("with_imu")
    with_joy = LaunchConfiguration("with_joy")
    with_hardware = LaunchConfiguration("with_hardware")
    with_controller = LaunchConfiguration("with_controller")

    # 1. IMU Driver Node
    imu_node = Node(
        package="serial_imu",
        executable="talker",
        name="serial_imu_talker",
        output="screen",
        condition=IfCondition(with_imu),
    )

    # 2. Joystick Teleop Node
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[{
            "dev": "/dev/input/js0",
            "deadzone": 0.05,
            "autorepeat_rate": 20.0,
        }],
        condition=IfCondition(with_joy),
    )

    # 3. RS00 CAN Hardware Driver Node
    hardware_node = Node(
        package="jaguar_control",
        executable="can_hardware_node.py",
        name="jaguar_can_hardware",
        output="screen",
        condition=IfCondition(with_hardware),
    )

    # 4. NXP Jaguar Sim-to-Real RL Controller Node
    controller_node = Node(
        package="jaguar_control",
        executable="nxp_jaguar_controller.py",
        name="nxp_jaguar_controller",
        output="screen",
        parameters=[{
            "policy_path": policy_path,
        }],
        condition=IfCondition(with_controller),
    )

    return LaunchDescription([
        policy_path_arg,
        with_imu_arg,
        with_joy_arg,
        with_hardware_arg,
        with_controller_arg,
        imu_node,
        joy_node,
        hardware_node,
        controller_node,
    ])
