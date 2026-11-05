"""Offline checks for shared configuration and body command bounds."""

import ast
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import numpy as np

ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from motion_limits import declare_motion_limits


def method(filename, name, **namespace):
    tree = ast.parse((ROOT / "scripts" / filename).read_text())
    function = next(n for cls in tree.body if isinstance(cls, ast.ClassDef)
                    for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == name)
    exec(compile(ast.Module(body=[function], type_ignores=[]), filename, "exec"), namespace)
    return namespace[name]


class FakeNode:
    def __init__(self, **overrides):
        self.values = overrides

    def declare_parameter(self, name, default):
        self.values.setdefault(name, default)

    def get_parameter(self, name):
        return SimpleNamespace(value=self.values[name])


class MotionLimitsTests(unittest.TestCase):
    def test_shared_config_is_all_one_and_ros_overrides_apply(self):
        limits = declare_motion_limits(FakeNode())
        self.assertEqual(len(limits), 5)
        self.assertTrue(all(value == 1.0 for value in limits.values()))
        self.assertEqual(declare_motion_limits(FakeNode(max_vx=0.4))["max_vx"], 0.4)

    def test_invalid_limits_rejected(self):
        for value in (0.0, -1.0, math.nan, math.inf):
            with self.subTest(value=value), self.assertRaises(ValueError):
                declare_motion_limits(FakeNode(linear_accel_limit=value))

    def test_mux_clamps_both_sources_and_rejects_nonfinite_values(self):
        clamp = method("jaguar_cmd_vel_mux.py", "_clamp_command", math=math, Twist=object)
        node = SimpleNamespace(motion_limits=declare_motion_limits(FakeNode()))
        msg = SimpleNamespace(linear=SimpleNamespace(x=3.0, y=-3.0),
                              angular=SimpleNamespace(z=math.nan))
        clamp(node, msg)
        self.assertEqual((msg.linear.x, msg.linear.y, msg.angular.z), (1.0, -1.0, 0.0))

    def test_controller_clamps_regular_and_benchmark_commands(self):
        import threading
        for callback in ("_cmd_vel_cb", "_benchmark_cmd_vel_cb"):
            clock = Mock()
            clock.now.return_value.nanoseconds = 1_000_000_000
            node = SimpleNamespace(motion_limits=declare_motion_limits(FakeNode()),
                                   state_lock=threading.Lock(), cmd_vel=np.zeros(3),
                                   cmd_timeout=0.25, last_benchmark_cmd_time=0.0,
                                   get_clock=lambda: clock)
            function = method("nxp_jaguar_controller.py", callback,
                              np=np, time=SimpleNamespace(monotonic=lambda: 10.0), Twist=object)
            function(node, SimpleNamespace(linear=SimpleNamespace(x=3.0, y=-3.0),
                                           angular=SimpleNamespace(z=2.0)))
            np.testing.assert_allclose(node.cmd_vel, [1.0, -1.0, 1.0])

    def test_keyboard_diagonal_and_yaw_ramp_use_configured_limits(self):
        import threading
        def twist():
            return SimpleNamespace(linear=SimpleNamespace(x=0.0, y=0.0),
                                   angular=SimpleNamespace(z=0.0))
        publish = method("keyboard_cmd_vel.py", "_publish", np=np, Twist=twist,
                         String=lambda: SimpleNamespace(),
                         time=SimpleNamespace(monotonic=lambda: 10.0))
        node = SimpleNamespace(lock=threading.Lock(), input_backend="TEST", pressed_keys={"w", "a"},
                               desired=np.ones(3), current=np.zeros(3), acceleration=1.0,
                               angular_acceleration=1.0, cmd_pub=Mock(), status_pub=Mock(),
                               joystick_present=lambda: False, target_linear=1.0, target_yaw=1.0)
        publish(node)
        self.assertAlmostEqual(float(np.linalg.norm(node.current[:2])), 0.02)
        self.assertAlmostEqual(node.current[2], 0.02)


if __name__ == "__main__":
    unittest.main()
