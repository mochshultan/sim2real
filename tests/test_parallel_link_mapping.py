"""Offline tests for the affine parallel-link coordinate mapping."""

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from parallel_link_mapping import motor_to_virtual_delta, virtual_to_motor_delta


class ParallelLinkMappingTests(unittest.TestCase):
    def test_hip_motion_does_not_change_virtual_knee(self):
        hip_delta, knee_delta = motor_to_virtual_delta(0.12, 0.0)
        self.assertAlmostEqual(hip_delta, 0.12)
        self.assertAlmostEqual(knee_delta, 0.0)

    def test_virtual_knee_maps_only_to_knee_motor(self):
        hip_motor_delta, knee_motor_delta = virtual_to_motor_delta(0.10, -0.03)
        self.assertAlmostEqual(hip_motor_delta, 0.10)
        self.assertAlmostEqual(knee_motor_delta, -0.03)

    def test_mapping_round_trip_with_non_unit_ratio(self):
        motor_delta = virtual_to_motor_delta(-0.08, 0.04, knee_ratio=0.95)
        virtual_delta = motor_to_virtual_delta(*motor_delta, knee_ratio=0.95)
        self.assertAlmostEqual(virtual_delta[0], -0.08)
        self.assertAlmostEqual(virtual_delta[1], 0.04)


if __name__ == "__main__":
    unittest.main()
