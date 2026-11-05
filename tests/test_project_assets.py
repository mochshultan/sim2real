"""Asset paths must work in source and installed layouts without hardware."""

import importlib.util
from pathlib import Path
import re
import sys
from types import SimpleNamespace
from unittest.mock import patch
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))
import project_paths
import parameters as hardware
from joint_layout import ISAAC_JOINT_NAMES, ROS_JOINT_NAMES, ROS_TO_ISAAC, NAME_TO_ROS_INDEX
from gamepad_reader import load_gamepad_calibration
from launch import LaunchContext


ROOT = Path(__file__).parents[1]


def test_installed_module_reads_package_share_and_user_calibration(tmp_path):
    fake_index = SimpleNamespace(get_package_share_directory=lambda name: str(tmp_path / "share"))
    with patch.object(project_paths, "__file__", str(tmp_path / "lib/jaguar_control/project_paths.py")), \
            patch.dict(sys.modules, {"ament_index_python.packages": fake_index}), \
            patch.dict("os.environ", {"XDG_CONFIG_HOME": str(tmp_path / "user")}):
        assert project_paths.package_file("models", "policy.pt") == tmp_path / "share/models/policy.pt"
        default_profile = tmp_path / "share/config/gamepad_calibration.json"
        assert project_paths.gamepad_calibration_file() == default_profile
        default_profile.parent.mkdir(parents=True)
        default_profile.write_text('{"profile": "factory"}')
        assert load_gamepad_calibration() == {"profile": "factory"}
        user_profile = project_paths.user_gamepad_calibration()
        user_profile.parent.mkdir(parents=True)
        user_profile.write_text('{"profile": "user"}')
        assert load_gamepad_calibration() == {"profile": "user"}


def test_urdf_mesh_references_resolve_in_installed_package():
    robot = ET.parse(ROOT / "urdf/nxp_jaguar.urdf")
    meshes = robot.findall(".//mesh")
    assert meshes
    for mesh in meshes:
        uri = mesh.attrib["filename"]
        assert uri.startswith("package://jaguar_control/")
        assert (ROOT / uri.removeprefix("package://jaguar_control/")).is_file()


def test_display_launch_honors_model_override(tmp_path):
    spec = importlib.util.spec_from_file_location("display_launch", ROOT / "launch/display.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    model = tmp_path / "custom.urdf"
    model.write_text('<robot name="override"/>')
    context = LaunchContext()
    context.launch_configurations["model"] = str(model)
    with patch.object(module, "Node") as node:
        module._launch_display(context)
    assert node.call_args_list[0].kwargs["parameters"] == [{"robot_description": model.read_text()}]


def test_python_and_cpp_motor_mapping_and_offset_conventions_agree():
    header = (ROOT / "include/jaguar_control/robstride_hardware_manager.hpp").read_text()
    entries = re.findall(
        r'joint_configs_\[(\d+)\]\s*=\s*\{"([^"]+)",\s*"([^"]+)",'
        r'\s*(\d+),\s*(-?\d+),\s*([+\-\d.]+),', header,
    )
    assert len(entries) == 12
    for index, name, bus, motor_id, direction, offset in entries:
        index = int(index)
        assert name == hardware.JOINT_NAME[index]
        assert bus == hardware.DEVICE[index]
        assert int(motor_id) == hardware.CAN_ID[index]
        assert int(direction) == hardware.MOTOR_DIR[index]
        assert float(offset) == -hardware.MOTOR_OFFSET_ANGLE[index]


def test_joint_aliases_preserve_hardware_and_policy_order():
    assert sorted(ROS_TO_ISAAC) == list(range(12))
    for isaac_index, ros_index in enumerate(ROS_TO_ISAAC):
        assert NAME_TO_ROS_INDEX[ISAAC_JOINT_NAMES[isaac_index]] == ros_index
        assert NAME_TO_ROS_INDEX[ROS_JOINT_NAMES[ros_index]] == ros_index
