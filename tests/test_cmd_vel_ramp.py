"""Exercise the real mux publish method without requiring ROS."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock


class Twist:
    def __init__(self):
        self.linear = SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.angular = SimpleNamespace(x=0.0, y=0.0, z=0.0)


source = ast.parse((Path(__file__).parents[1] / "scripts/jaguar_cmd_vel_mux.py").read_text())
method = next(n for cls in source.body if isinstance(cls, ast.ClassDef)
              for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "_publish")
clock = SimpleNamespace(monotonic=Mock())
namespace = dict(time=clock, math=math, Twist=Twist)
exec(compile(ast.Module(body=[method], type_ignores=[]), "<mux publish>", "exec"), namespace)


class RampTests(unittest.TestCase):
    def setUp(self):
        self.node = SimpleNamespace(
            xbox_cmd=Twist(), keyboard_cmd=Twist(), smoothed_xbox=Twist(),
            xbox_stamp=10.0, keyboard_stamp=0.0, last_publish_time=10.0,
            xbox_timeout=0.3, keyboard_timeout=0.3,
            linear_accel=0.5, angular_accel=1.0, output=Mock())

    def tick(self, now, heartbeat=True):
        if heartbeat:
            self.node.xbox_stamp = now
        clock.monotonic.return_value = now
        namespace["_publish"](self.node)
        return self.node.output.publish.call_args.args[0]

    def test_step_to_one_takes_two_seconds_and_stays_at_target(self):
        self.node.xbox_cmd.linear.x = 1.0
        self.assertAlmostEqual(self.tick(10.02).linear.x, 0.01)
        for i in range(2, 102):
            cmd = self.tick(10.0 + i * 0.02)
            self.assertAlmostEqual(cmd.linear.x, min(i * 0.01, 1.0))

    def test_diagonal_and_yaw_are_bounded(self):
        self.node.xbox_cmd.linear.x = self.node.xbox_cmd.linear.y = 1.0
        self.node.xbox_cmd.angular.z = -1.0
        cmd = self.tick(10.02)
        self.assertAlmostEqual(math.hypot(cmd.linear.x, cmd.linear.y), 0.01)
        self.assertAlmostEqual(cmd.angular.z, -0.02)

    def test_release_and_reversal_are_smoothed(self):
        self.node.smoothed_xbox.linear.x = 0.5
        self.assertAlmostEqual(self.tick(10.02).linear.x, 0.49)
        self.node.xbox_cmd.linear.x = -1.0
        self.assertAlmostEqual(self.tick(10.04).linear.x, 0.48)

    def test_timeout_stops_immediately_and_reconnect_starts_from_zero(self):
        self.node.smoothed_xbox.linear.x = 0.8
        self.assertEqual(self.tick(10.4, heartbeat=False).linear.x, 0.0)
        self.node.xbox_cmd.linear.x = 1.0
        self.assertAlmostEqual(self.tick(10.42).linear.x, 0.01)

    def test_keyboard_fallback_and_timer_delay(self):
        self.node.keyboard_stamp = 10.4
        self.node.keyboard_cmd.linear.x = 0.3
        self.assertEqual(self.tick(10.4, heartbeat=False).linear.x, 0.3)
        self.node.xbox_cmd.linear.x = 1.0
        self.assertAlmostEqual(self.tick(12.0).linear.x, 0.01)


if __name__ == "__main__":
    unittest.main()
