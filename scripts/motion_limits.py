"""Shared body command limits from sim2real.yaml, with ROS parameter overrides."""

import math
from project_paths import load_default_config


def declare_motion_limits(node):
    defaults = load_default_config()["/**"]["ros__parameters"]
    limits = {}
    for name in ("max_vx", "max_vy", "max_wz", "linear_accel_limit", "angular_accel_limit"):
        node.declare_parameter(name, float(defaults[name]))
        value = float(node.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
        limits[name] = value
    return limits
