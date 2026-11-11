"""The launched resource contract must cover all joints and preserve safety wiring."""

import importlib.util
from pathlib import Path
from unittest.mock import patch
import xml.etree.ElementTree as ET

from launch import LaunchContext
import yaml

ROOT = Path(__file__).parents[1]
spec = importlib.util.spec_from_file_location("control_launch", ROOT / "launch/sim2real.launch.py")
launch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch)


def test_generated_resources_use_physical_urdf_names_and_five_impedance_commands():
    robot = ET.fromstring(launch._robot_description(ROOT, {"mock_hardware": True}))
    physical = {joint.attrib["name"] for joint in robot.findall("joint") if joint.attrib["type"] != "fixed"}
    control = robot.find("ros2_control")
    assert control.findtext("hardware/plugin") == "jaguar_control/JaguarSystem"
    assert control.findtext("hardware/param[@name='mock_hardware']") == "true"
    joints = control.findall("joint")
    assert len(joints) == 12
    assert {joint.attrib["name"] for joint in joints} == physical
    for joint in joints:
        assert {interface.attrib["name"] for interface in joint.findall("command_interface")} == {
            "position", "velocity", "effort", "kp", "kd"}
        assert {interface.attrib["name"] for interface in joint.findall("state_interface")} == {
            "position", "velocity", "effort", "temperature"}
    safety = control.find("gpio")
    assert safety.attrib["name"] == "jaguar_safety"
    assert {interface.attrib["name"] for interface in safety.findall("command_interface")} == {
        "emergency_stop", "safe_park_active", "reset_fault"}


def test_launch_selects_controller_manager_and_preserves_cap_and_mock_setting(tmp_path):
    config = yaml.safe_load((ROOT / "config/sim2real.yaml").read_text())
    config["jaguar_hardware"]["ros__parameters"]["control_torque_limit_nm"] = 3.0
    path = tmp_path / "custom.yaml"
    path.write_text(yaml.safe_dump(config))
    context = LaunchContext()
    context.launch_configurations.update(
        config_file=str(path), mock_hardware="true", startup_clear_faults="false")
    with patch.object(launch, "Node") as node:
        launch._launch_hardware(context)
    calls = [call.kwargs for call in node.call_args_list]
    assert [call["executable"] for call in calls] == ["robot_state_publisher", "ros2_control_node", "spawner"]
    robot = ET.fromstring(calls[0]["parameters"][0]["robot_description"])
    parameters = {param.attrib["name"]: param.text for param in robot.findall("ros2_control/hardware/param")}
    assert parameters["mock_hardware"] == "true"
    assert parameters["startup_clear_faults"] == "false"
    assert parameters["control_torque_limit_nm"] == "3.0"
    assert Path(parameters["motor_params_file"]).is_file()
    assert calls[1]["parameters"] == [str(path)]
    assert calls[2]["arguments"][:2] == ["joint_state_broadcaster", "jaguar_impedance_controller"]


def test_yaml_keeps_controller_and_can_rates_separate_and_torque_diagnostic_only():
    config = yaml.safe_load((ROOT / "config/sim2real.yaml").read_text())
    manager = config["controller_manager"]["ros__parameters"]
    assert manager["update_rate"] == 200
    assert manager["jaguar_impedance_controller"]["type"] == "jaguar_control/ImpedanceController"
    assert "torque_safety_enabled" not in config["nxp_jaguar_controller"]["ros__parameters"]
    assert config["jaguar_hardware"]["ros__parameters"]["control_torque_limit_nm"] == 14.0
