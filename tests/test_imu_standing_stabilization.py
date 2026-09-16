"""Unit tests for bounded standing IMU stabilization."""

import ast
import math
from pathlib import Path
import time
import unittest
from unittest.mock import patch

import numpy as np


source = ast.parse((Path(__file__).parents[1] / "scripts/nxp_jaguar_controller.py").read_text())
controller_class = next(
    node for node in source.body
    if isinstance(node, ast.ClassDef) and node.name == "NXPJaguarControllerNode"
)
method = next(
    node for node in controller_class.body
    if isinstance(node, ast.FunctionDef) and node.name == "_compute_imu_stabilization"
)
controller_class.bases = []
controller_class.body = [method]
namespace = {"np": np, "math": math, "time": time, "N_JOINTS": 12}
exec(
    compile(ast.Module(body=[controller_class], type_ignores=[]), "<imu stabilization>", "exec"),
    namespace,
)
Controller = namespace["NXPJaguarControllerNode"]


class ImuStandingStabilizationTests(unittest.TestCase):
    def setUp(self):
        self.node = Controller()
        self.node.use_imu_stabilization = True
        self.node.imu_received = True
        self.node.last_imu_time = 9.9
        self.node.body_quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
        self.node.body_ang_vel = np.zeros(3, dtype=np.float32)
        self.node.imu_kp_pitch = 0.55
        self.node.imu_kd_pitch = 0.04
        self.node.imu_kp_roll = 0.30
        self.node.imu_kd_roll = 0.03
        self.node.imu_hip_tolerance = 0.15
        self.node.imu_roll_tolerance = 0.12
        self.node.imu_tilt_deadband = 0.02

    def compute(self, alpha=1.0, state="STAND_HOLD"):
        with patch.object(time, "monotonic", return_value=10.0):
            return self.node._compute_imu_stabilization(alpha, state)

    def test_upright_robot_has_no_trim(self):
        np.testing.assert_allclose(self.compute(), np.zeros(12), atol=1e-7)

    def test_hip_trim_is_bounded_and_knees_never_change(self):
        angle = 0.50
        self.node.body_quat = np.array(
            [0.0, math.sin(angle / 2.0), 0.0, math.cos(angle / 2.0)],
            dtype=np.float32,
        )
        trim = self.compute()
        self.assertLessEqual(float(np.max(np.abs(trim[4:8]))), 0.150001)
        self.assertGreater(float(np.max(np.abs(trim[4:8]))), 0.0)
        np.testing.assert_array_equal(trim[8:12], np.zeros(4))

    def test_roll_trim_is_bounded_and_alternates_left_right(self):
        angle = 0.50
        self.node.body_quat = np.array(
            [math.sin(angle / 2.0), 0.0, 0.0, math.cos(angle / 2.0)],
            dtype=np.float32,
        )
        trim = self.compute()
        self.assertLessEqual(float(np.max(np.abs(trim[0:4]))), 0.120001)
        np.testing.assert_allclose(trim[0:4], [trim[0], -trim[0], trim[0], -trim[0]])
        np.testing.assert_array_equal(trim[8:12], np.zeros(4))

    def test_transition_ramps_in_and_other_states_are_disabled(self):
        self.node.body_quat = np.array([0.0, 0.1, 0.0, math.sqrt(0.99)], dtype=np.float32)
        np.testing.assert_array_equal(self.compute(alpha=0.15, state="STANDUP"), np.zeros(12))
        self.assertGreater(float(np.max(np.abs(self.compute(alpha=0.50, state="STANDUP")))), 0.0)
        np.testing.assert_array_equal(self.compute(state="WALK"), np.zeros(12))

    def test_stale_imu_disables_trim(self):
        self.node.last_imu_time = 9.0
        self.node.body_quat = np.array([0.0, 0.1, 0.0, math.sqrt(0.99)], dtype=np.float32)
        np.testing.assert_array_equal(self.compute(), np.zeros(12))


if __name__ == "__main__":
    unittest.main()
