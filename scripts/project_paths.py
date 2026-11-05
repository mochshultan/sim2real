"""Resolve Jaguar assets consistently from source and installed ROS packages."""

import os
from pathlib import Path

import yaml


def package_file(*parts: str) -> Path:
    source_root = Path(__file__).absolute().parents[1]
    if (source_root / "config/sim2real.yaml").is_file():
        return source_root.joinpath(*parts)

    from ament_index_python.packages import get_package_share_directory

    return Path(get_package_share_directory("jaguar_control")).joinpath(*parts)


def load_default_config() -> dict:
    with package_file("config", "sim2real.yaml").open() as stream:
        return yaml.safe_load(stream)


def user_gamepad_calibration() -> Path:
    config_home = Path(os.environ.get("XDG_CONFIG_HOME", Path.home() / ".config"))
    return config_home / "jaguar_control" / "gamepad_calibration.json"


def gamepad_calibration_file() -> Path:
    user_profile = user_gamepad_calibration()
    if user_profile.is_file():
        return user_profile
    return package_file("config", "gamepad_calibration.json")
