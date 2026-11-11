"""Shared Jaguar hardware, teleop and controller stack for both policy references."""

import os
import xml.etree.ElementTree as ET

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def _launch_controller(context):
    overrides = {
        "policy_path": LaunchConfiguration("policy_path"),
        "policy_reference": LaunchConfiguration("policy_reference"),
        "history_initialization": LaunchConfiguration("history_initialization"),
    }
    return [Node(
        package="jaguar_control",
        executable="nxp_jaguar_controller.py",
        name="nxp_jaguar_controller",
        output="screen",
        parameters=[LaunchConfiguration("config_file"), overrides],
    )]


def _robot_description(package_dir, hardware_parameters):
    """Add the ros2_control contract to the existing calibrated Jaguar URDF."""
    root = ET.parse(os.path.join(package_dir, "urdf", "nxp_jaguar.urdf")).getroot()
    system = ET.SubElement(root, "ros2_control", name="JaguarRS00", type="system")
    hardware = ET.SubElement(system, "hardware")
    ET.SubElement(hardware, "plugin").text = "jaguar_control/JaguarSystem"
    for name, value in hardware_parameters.items():
        ET.SubElement(hardware, "param", name=name).text = str(value).lower() if isinstance(value, bool) else str(value)
    for leg in ("Bl", "Br", "Fl", "Fr"):
        for axis in ("roll", "hip_pitch", "knee"):
            joint = ET.SubElement(system, "joint", name=f"{leg}_{axis}_joint")
            for interface in ("position", "velocity", "kp", "kd", "effort"):
                ET.SubElement(joint, "command_interface", name=interface)
            for interface in ("position", "velocity", "effort", "temperature"):
                state = ET.SubElement(joint, "state_interface", name=interface)
                ET.SubElement(state, "param", name="initial_value").text = "25" if interface == "temperature" else "0"
    safety = ET.SubElement(system, "gpio", name="jaguar_safety")
    for interface in ("emergency_stop", "safe_park_active", "reset_fault"):
        ET.SubElement(safety, "command_interface", name=interface)
    for interface in ("emergency_stopped", "passive", "park_requested", "resetting", "torque_limit"):
        ET.SubElement(safety, "state_interface", name=interface)
    return ET.tostring(root, encoding="unicode")


def _launch_hardware(context):
    package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    config_file = LaunchConfiguration("config_file").perform(context)
    with open(config_file) as stream:
        config = yaml.safe_load(stream)
    hardware_parameters = dict(config["jaguar_hardware"]["ros__parameters"])
    hardware_parameters.setdefault("motor_params_file", os.path.join(package_dir, "config", "cybergear_motor_params.yaml"))
    hardware_parameters["mock_hardware"] = LaunchConfiguration("mock_hardware").perform(context)
    hardware_parameters["startup_clear_faults"] = LaunchConfiguration("startup_clear_faults").perform(context)
    description = _robot_description(package_dir, hardware_parameters)
    return [
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            parameters=[{"robot_description": description}], output="screen",
        ),
        Node(
            package="controller_manager", executable="ros2_control_node",
            parameters=[config_file], output="screen",
            remappings=[("~/robot_description", "/robot_description")],
        ),
        Node(
            package="controller_manager", executable="spawner",
            arguments=["joint_state_broadcaster", "jaguar_impedance_controller",
                       "--controller-manager", "/controller_manager",
                       "--controller-manager-timeout", "60", "--activate-as-group"],
            output="screen",
        ),
    ]


def generate_launch_description():
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    default_policy = os.path.join(pkg_dir, "models", "policy.pt")
    default_config = os.path.join(pkg_dir, "config", "sim2real.yaml")

    # Launch arguments
    domain_id_arg = DeclareLaunchArgument(
        "domain_id",
        default_value=EnvironmentVariable("ROS_DOMAIN_ID", default_value="1"),
        description="Dedicated robot DDS domain; use the same ID in every teleop/CLI terminal",
    )
    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=default_config,
        description="Path to ROS 2 parameters YAML configuration file",
    )
    policy_path_arg = DeclareLaunchArgument(
        "policy_path",
        default_value=default_policy,
        description="Path to TorchScript policy; defaults to models/policy.pt",
    )
    policy_reference_arg = DeclareLaunchArgument(
        "policy_reference",
        default_value="nominal",
        choices=["nominal", "zero"],
        description="Joint reference for policy observations and position targets",
    )
    history_initialization_arg = DeclareLaunchArgument(
        "history_initialization", default_value="zeros", choices=["zeros", "repeat"],
        description="History cold start required by the selected policy",
    )
    with_imu_arg = DeclareLaunchArgument(
        "with_imu",
        default_value="true",
        description="Launch serial_imu node",
    )
    with_joy_arg = DeclareLaunchArgument(
        "with_joy",
        default_value="false",
        description="Launch optional joy_node remapped to /joy_raw",
    )
    with_teleop_arg = DeclareLaunchArgument(
        "with_teleop",
        default_value="true",
        description="Launch Xbox teleoperation source",
    )
    with_hardware_arg = DeclareLaunchArgument(
        "with_hardware",
        default_value="true",
        description="Launch controller_manager, Jaguar hardware and impedance controller",
    )
    mock_hardware_arg = DeclareLaunchArgument(
        "mock_hardware",
        default_value="false",
        choices=["true", "false"],
        description="Use offline hardware with no CAN access (not a dynamics simulator)",
    )
    with_controller_arg = DeclareLaunchArgument(
        "with_controller",
        default_value="true",
        description="Launch NXP Jaguar RL controller node",
    )
    startup_clear_faults_arg = DeclareLaunchArgument(
        "startup_clear_faults",
        default_value="true",
        description="Clear latched RobStride faults once before hardware activation",
    )
    config_file = LaunchConfiguration("config_file")
    with_imu = LaunchConfiguration("with_imu")
    with_joy = LaunchConfiguration("with_joy")
    with_teleop = LaunchConfiguration("with_teleop")
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

    # 2. Raw joystick source for keyboard_teleop. Keep raw events away from the
    # controller's normalized /joy mode topic.
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[config_file],
        remappings=[("/joy", "/joy_raw")],
        condition=IfCondition(with_joy),
    )

    teleop_node = Node(
        package="jaguar_control",
        executable="keyboard_teleop.py",
        name="jaguar_unified_teleop",
        output="screen",
        parameters=[config_file, {"keyboard_enabled": False}],
        remappings=[("/cmd_vel", "/cmd_vel/xbox")],
        condition=IfCondition(with_teleop),
    )

    # Local mux: Xbox is preferred while its command heartbeat is fresh. A
    # keyboard source is accepted only when keyboard_cmd_vel.py is started
    # manually in a separate terminal; it is intentionally not auto-launched
    # so this terminal remains dedicated to important controller/hardware logs.
    cmd_vel_mux_node = Node(
        package="jaguar_control",
        executable="jaguar_cmd_vel_mux.py",
        name="jaguar_cmd_vel_mux",
        output="screen",
        parameters=[config_file],
    )

    # controller_manager owns read/update/write; there is no standalone CAN loop.
    hardware_stack = OpaqueFunction(
        function=_launch_hardware,
        condition=IfCondition(with_hardware),
    )

    # 4. NXP Jaguar Sim-to-Real RL Controller Node
    controller_node = OpaqueFunction(
        function=_launch_controller,
        condition=IfCondition(with_controller),
    )

    return LaunchDescription([
        domain_id_arg,
        SetEnvironmentVariable("ROS_DOMAIN_ID", LaunchConfiguration("domain_id")),
        LogInfo(msg=["Jaguar DDS domain: ", LaunchConfiguration("domain_id"),
                     "; all remote nodes and CLI terminals must use this domain and matching ROS distro."]),
        config_file_arg,
        policy_path_arg,
        policy_reference_arg,
        history_initialization_arg,
        with_imu_arg,
        with_joy_arg,
        with_teleop_arg,
        with_hardware_arg,
        mock_hardware_arg,
        with_controller_arg,
        startup_clear_faults_arg,
        imu_node,
        joy_node,
        teleop_node,
        cmd_vel_mux_node,
        hardware_stack,
        controller_node,
    ])
