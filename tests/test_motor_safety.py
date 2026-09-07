"""Offline tests only: fake CAN transport, no sockets or actuator constructors."""
import math
from pathlib import Path
import struct
import sys
import time
import unittest
from unittest.mock import patch

import can

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from robstride_motor_lib import RobStrideMotorController


def feedback(position=0.0, motor_id=1, mode=2, host=254, flags=0):
    raw = int((position + 4 * math.pi) / (8 * math.pi) * 65535)
    return can.Message(arbitration_id=(mode << 24) | (motor_id << 8) | host | flags,
                       is_extended_id=True, data=struct.pack(">HHHH", raw, 32767, 32767, 300))


class FakeBus:
    def __init__(self):
        self.sent = []
        self.pending = []
        self.response = [feedback()]

    def send(self, message, timeout=None):
        self.sent.append(message)
        self.pending.extend(self.response)

    def recv(self, timeout=None):
        return self.pending.pop(0) if self.pending else None


class MotorTests(unittest.TestCase):
    def setUp(self):
        self.bus = FakeBus()
        with patch.dict(RobStrideMotorController.bus, {"test": self.bus}):
            self.motor = RobStrideMotorController(bus="test", motor_id=1)
        self.registry = patch.dict(RobStrideMotorController.bus, {"test": self.bus})
        self.registry.start()
        self.addCleanup(self.registry.stop)
        self.bus.sent.clear()

    def test_frame_routing(self):
        remote = feedback()
        remote.is_remote_frame = True
        standard = feedback()
        standard.is_extended_id = False
        self.bus.response = [feedback(motor_id=6), feedback(mode=0), feedback(mode=1),
                             feedback(host=253), remote, standard, feedback()]
        result = self.motor.send_control_command(0, 0, 0, 0, 0, timeout=15)
        self.assertEqual(result[0], 1)
        self.assertTrue(self.motor.feedback_fresh())

    def test_short_frame_invalidates(self):
        self.assertEqual(self.motor.parse_received_msg(b"\x00", feedback().arbitration_id), (None,) * 5)
        self.assertFalse(self.motor.feedback_fresh())

    def test_unrelated_payloads_never_decode(self):
        for frame in (feedback(motor_id=6), feedback(mode=0), feedback(mode=1), feedback(host=253)):
            self.assertEqual(self.motor.parse_received_msg(frame.data, frame.arbitration_id), (None,) * 5)

    def test_raw_position_not_wrapped_or_arbitrarily_clipped(self):
        for position in (2.6, 12.0, -1.0):
            self.motor.feedback_valid = False
            frame = feedback(position)
            result = self.motor.parse_received_msg(frame.data, frame.arbitration_id)
            self.assertAlmostEqual(result[1], position, delta=0.001)

    def test_jump_latches_and_blocks_positive_gains(self):
        frame = feedback(12)
        self.motor.parse_received_msg(frame.data, frame.arbitration_id)
        self.motor.send_control_command(0, 0, 28, 0.7, 0)
        self.assertIsNotNone(self.motor.fault_reason)
        self.assertTrue(all((m.arbitration_id >> 24) == 4 for m in self.bus.sent))

    def test_stale_feedback_blocks_activation(self):
        self.motor.last_feedback_time = time.monotonic() - 1
        self.motor.send_control_command(0, 0, 28, 0.7, 0)
        self.assertEqual(self.bus.sent[0].arbitration_id >> 24, 4)

    def test_missing_active_reply_requests_stop(self):
        self.bus.response = []
        self.motor.send_control_command(0, 0, 28, 0.7, 0, timeout=10)
        self.assertEqual([m.arbitration_id >> 24 for m in self.bus.sent], [1, 4])
        self.assertIsNotNone(self.motor.fault_reason)

    def test_fault_bit_latches(self):
        frame = feedback(flags=1 << 16)
        self.motor.parse_received_msg(frame.data, frame.arbitration_id)
        self.assertIsNotNone(self.motor.fault_reason)
        self.assertFalse(self.motor.feedback_fresh())

    def test_nonfinite_command_requests_stop(self):
        self.motor.send_control_command(float("nan"), 0, 28, 0.7, 0)
        self.assertEqual(self.bus.sent[0].arbitration_id >> 24, 4)

    def test_initial_position_outside_joint_limits_blocks_activation(self):
        self.motor.set_angle_range(-0.5, 0.5)
        self.motor.feedback_valid = False
        frame = feedback(12)
        self.motor.parse_received_msg(frame.data, frame.arbitration_id)
        self.motor.send_control_command(0, 0, 28, 0.7, 0)
        self.assertEqual(self.bus.sent[0].arbitration_id >> 24, 4)

    def test_calibration_change_resets_feedback_baseline(self):
        self.motor.set_angle_offset(1.35)
        self.assertFalse(self.motor.feedback_fresh())
        self.motor.send_control_command(0, 0, 0, 0, 0)
        self.assertIsNone(self.motor.fault_reason)
        self.assertAlmostEqual(self.motor.pos_cur, 1.35, delta=0.001)

    def test_enable_has_eight_bytes(self):
        self.motor.enable_motor()
        self.assertEqual(self.bus.sent[-1].dlc, 8)


if __name__ == "__main__":
    unittest.main()
