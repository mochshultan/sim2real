"""Exercise real controller safety methods without ROS, policy loading, or CAN."""
import ast
from pathlib import Path
import time
import unittest
from unittest.mock import Mock, patch
import numpy as np

source = ast.parse((Path(__file__).parents[1] / 'scripts/nxp_jaguar_controller.py').read_text())
cls = next(n for n in source.body if isinstance(n, ast.ClassDef) and n.name == 'NXPJaguarControllerNode')
methods = {'_trigger_safe_shutdown', '_persistent_fault', '_hardware_safe_park_cb',
           '_request_safe_park', '_sensors_ready', '_joy_cb'}
cls.bases = []
cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in methods]
namespace = dict(np=np, time=time, Bool=Mock, Joy=Mock,
                 ISAAC_LIMITS_LOWER=np.full(12, -0.5), ISAAC_LIMITS_UPPER=np.full(12, 0.5),
                 FEEDBACK_LIMITS_LOWER=np.full(12, -0.7), FEEDBACK_LIMITS_UPPER=np.full(12, 0.7),
                 SIT_JOINT_POS=np.zeros(12))
exec(compile(ast.Module(body=[cls], type_ignores=[]), '<controller safety methods>', 'exec'), namespace)
Controller = namespace['NXPJaguarControllerNode']


class SafetyTests(unittest.TestCase):
    def setUp(self):
        self.node = Controller()
        self.node.state = 'WALK'
        self.node.soft_fault_times = {}
        self.node._trigger_hard_estop = Mock()
        self.node._request_safe_park = Mock()
        self.node._sensors_ready = Mock(return_value=True)

    def test_recoverable_fault_parks_then_escalates_during_park(self):
        self.node._trigger_safe_shutdown(10.0, 'tracking')
        self.node._request_safe_park.assert_called_once_with(10.0, 'tracking', automatic=True)
        self.node._trigger_hard_estop.assert_not_called()
        self.node.state = 'SAFE_PARK'
        self.node._trigger_safe_shutdown(10.1, 'tracking')
        self.node._trigger_hard_estop.assert_called_once_with('tracking')

    def test_unusable_feedback_cannot_park(self):
        self.node._sensors_ready.return_value = False
        self.node._trigger_safe_shutdown(10.0, 'feedback')
        self.node._request_safe_park.assert_not_called()
        self.node._trigger_hard_estop.assert_called_once()

    def test_debounce_resets_after_good_sample(self):
        with patch.object(time, 'monotonic', side_effect=[1.0, 1.05, 1.2, 1.31]):
            self.assertFalse(self.node._persistent_fault('tracking', True))
            self.assertFalse(self.node._persistent_fault('tracking', True))
            self.assertFalse(self.node._persistent_fault('tracking', False))
            self.assertFalse(self.node._persistent_fault('tracking', True))
            self.assertTrue(self.node._persistent_fault('tracking', True))

    def test_queued_driver_requests_do_not_restart_park(self):
        self.node.state = 'SAFE_PARK'
        self.node._hardware_safe_park_cb(Mock(data=True))
        self.node._request_safe_park.assert_not_called()
        self.node._trigger_hard_estop.assert_not_called()

    def test_held_safe_park_button_only_triggers_once(self):
        self.node.previous_buttons = []
        clock = Mock()
        clock.now.return_value.nanoseconds = 2_000_000_000
        self.node.get_clock = Mock(return_value=clock)
        msg = Mock(buttons=[0, 0, 0, 0, 0, 1, 0], axes=[0.8, -0.7, 0.0, 0.6])

        self.node._joy_cb(msg)
        self.node._joy_cb(msg)

        self.node._request_safe_park.assert_called_once_with(
            2.0, 'Xbox RB safe-park request')

    def test_joy_axes_cannot_overwrite_cmd_vel(self):
        self.node.previous_buttons = []
        self.node.cmd_vel = np.array([0.25, -0.35, 0.45], dtype=np.float32)
        self.node.last_cmd_time = 1.25
        clock = Mock()
        clock.now.return_value.nanoseconds = 2_000_000_000
        self.node.get_clock = Mock(return_value=clock)
        msg = Mock(buttons=[0, 0, 0], axes=[-1.0, 1.0, 0.0, -1.0])

        self.node._joy_cb(msg)

        np.testing.assert_allclose(self.node.cmd_vel, [0.25, -0.35, 0.45])
        self.assertEqual(self.node.last_cmd_time, 1.25)


    def test_recovery_envelope_and_feedback_freshness(self):
        node = self.node
        del node._sensors_ready
        node.imu_received = node.joints_received = True
        node.last_imu_time = node.last_joints_time = 1.0
        node.joint_pos = np.zeros(12)
        node.joint_pos[3] = -0.794
        with patch.object(time, 'monotonic', return_value=1.1):
            self.assertFalse(node._sensors_ready())
            self.assertTrue(node._sensors_ready(check_limits=False))
            node.joint_pos[3] = -0.91
            self.assertFalse(node._sensors_ready(check_limits=False))
        node.joint_pos[:] = 0.0
        with patch.object(time, 'monotonic', return_value=1.3):
            self.assertFalse(node._sensors_ready(check_limits=False))


if __name__ == '__main__':
    unittest.main()
