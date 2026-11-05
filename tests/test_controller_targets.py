"""Controller refactoring must preserve trajectories, policy blending and guards."""

from pathlib import Path
import sys
import threading
from types import SimpleNamespace
from unittest.mock import Mock
from types import MethodType
from unittest.mock import patch

import numpy as np
import pytest
import torch

sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))
from nxp_jaguar_controller import (
    NXPJaguarControllerNode as Controller, DEFAULT_JOINT_POS, STAND_JOINT_POS,
)
from policy_observation import JaguarObservationBuilder
import parameters as hardware
from project_paths import load_default_config


@pytest.mark.parametrize("enabled", [True, False])
def test_fourteen_nm_cap_accepts_independent_feedback_watchdog(enabled):
    values = {"torque_safety_enabled": enabled}

    def declare(name, default):
        values.setdefault(name, default)

    def parameter(name):
        value = values[name]
        return SimpleNamespace(
            value=value, get_parameter_value=lambda: SimpleNamespace(string_value=value),
        )

    node = SimpleNamespace(declare_parameter=declare, get_parameter=parameter, get_logger=Mock())
    Controller._configure_parameters(node)
    assert hardware.RS00_CONTROL_TORQUE_LIMIT_NM == 14.0
    assert load_default_config()["robstride_can_hardware"]["ros__parameters"]["control_torque_limit_nm"] == 14.0
    assert node.overtorque_threshold == 14.0
    assert node.torque_safety_enabled is enabled


@pytest.mark.parametrize("duration", [2.0, 4.0])
@pytest.mark.parametrize("fraction,blend", [(0.0, 0.0), (0.5, 0.5), (1.0, 1.0)])
def test_shared_transition_preserves_cosine_trajectory(duration, fraction, blend):
    node = SimpleNamespace(transition_start_time=10.0,
                           transition_start_pos=np.zeros(12),
                           transition_target_pos=np.ones(12))
    elapsed, alpha, position, velocity = Controller._transition_targets(
        node, 10.0 + duration * fraction, duration,
    )
    assert elapsed == duration * fraction
    assert alpha == fraction
    np.testing.assert_allclose(position, blend, atol=1e-7)
    expected_velocity = np.pi / (2 * duration) if fraction == 0.5 else 0.0
    np.testing.assert_allclose(velocity, expected_velocity, atol=1e-7)


def walk_node(action):
    return SimpleNamespace(
        obs_builder=JaguarObservationBuilder(DEFAULT_JOINT_POS),
        policy=lambda history: torch.tensor([action], dtype=torch.float32),
        policy_expects_temporal=True, action_ema_alpha=0.0,
        filtered_action=np.zeros(12), walk_start_time=10.0, walk_ramp_duration=0.8,
        rl_kp=[45.0] * 12, rl_kd=[4.5] * 12, transition_kd=[4.5] * 12,
        debug_pub=Mock(), _trigger_safe_shutdown=Mock(),
    )


@pytest.mark.parametrize("elapsed,ramp", [(0.0, 0.0), (0.4, 0.5), (0.8, 1.0)])
def test_walk_targets_preserve_policy_blend_and_gains(elapsed, ramp):
    node = walk_node([0.2] * 12)
    targets = Controller._walk_targets(node, 10.0 + elapsed, DEFAULT_JOINT_POS,
                                       np.zeros(12), [0, 0, 0, 1], np.zeros(3), np.zeros(3))
    position, velocity, kp, kd = targets
    expected = (1 - ramp) * STAND_JOINT_POS + ramp * DEFAULT_JOINT_POS + 0.25 * ramp * 0.2
    np.testing.assert_allclose(position, expected, atol=1e-7)
    np.testing.assert_array_equal(velocity, 0)
    assert kp == [45.0] * 12
    assert kd == [4.5] * 12


def test_invalid_walk_output_requests_shutdown_and_returns_no_target():
    node = walk_node([float("nan")] * 12)
    result = Controller._walk_targets(node, 11.0, DEFAULT_JOINT_POS,
                                      np.zeros(12), [0, 0, 0, 1], np.zeros(3), np.zeros(3))
    assert result is None
    node._trigger_safe_shutdown.assert_called_once_with(11.0, "code=POLICY_OUTPUT shape=12 expected_shape=12 finite=0")
    node.debug_pub.publish.assert_not_called()


def test_tracking_fault_blocks_publication_after_extraction():
    node = SimpleNamespace(state="WALK", joint_pos=np.full(12, 5.0),
                           _persistent_fault=Mock(return_value=True), _log_safety=Mock(),
                           soft_fault_times={"tracking": 0.0},
                           _trigger_safe_shutdown=Mock(), joint_cmd_pub=Mock())
    assert not Controller._publish_targets(node, np.zeros(12), np.zeros(12),
                                           [45.0] * 12, [4.5] * 12, 10.0)
    node._trigger_safe_shutdown.assert_called_once()
    node.joint_cmd_pub.publish.assert_not_called()
    assert node._log_safety.call_count == 11
    assert "code=TRACKING joint=Fr_roll_joint actual_rad=5.0000" in node._trigger_safe_shutdown.call_args.args[1]
    assert "limit_rad=2.0000" in node._trigger_safe_shutdown.call_args.args[1]


def test_estop_logs_new_causes_after_latching_without_republishing_stop():
    node = SimpleNamespace(state="WALK", get_logger=Mock(), state_lock=threading.Lock(),
                           cmd_vel=np.ones(3), estop_pub=Mock(), joint_cmd_pub=Mock())
    node._log_safety = MethodType(Controller._log_safety, node)
    Controller._trigger_hard_estop(node, "code=SENSOR_TIMEOUT sensor=imu age_s=0.260 limit_s=0.250")
    Controller._trigger_hard_estop(node, "code=SENSOR_TIMEOUT sensor=imu age_s=0.280 limit_s=0.250")
    Controller._trigger_hard_estop(node, "code=SENSOR_TIMEOUT sensor=joints age_s=0.270 limit_s=0.250")
    messages = [call.args[0] for call in node.get_logger.return_value.error.call_args_list]
    assert len(messages) == 2
    assert messages[0].startswith("event=ESTOP state=WALK code=SENSOR_TIMEOUT sensor=imu")
    assert messages[1].startswith("event=ESTOP state=DISABLED code=SENSOR_TIMEOUT sensor=joints")
    node.estop_pub.publish.assert_called_once()
    node.joint_cmd_pub.publish.assert_called_once()
    np.testing.assert_array_equal(node.cmd_vel, 0)


def test_sensor_fault_logs_distinguish_invalid_timeout_nonfinite_and_range():
    node = SimpleNamespace(imu_received=False, joints_received=True,
                           last_imu_time=9.0, last_joints_time=9.9,
                           joint_pos=np.zeros(12), imu_fault_detail="quat_norm_sq=0.000000")
    node.joint_pos[0] = float("nan")
    node.joint_pos[1] = 1.0
    with patch("nxp_jaguar_controller.time.monotonic", return_value=10.0):
        reasons = Controller._sensor_fault_reasons(node)
    assert len(reasons) == 4
    assert reasons[0] == "code=SENSOR_INVALID sensor=imu quat_norm_sq=0.000000"
    assert reasons[1] == "code=SENSOR_TIMEOUT sensor=imu age_s=1.000 limit_s=0.250"
    assert "code=POSITION_NONFINITE joint=Fr_roll_joint" in reasons[2]
    assert "code=POSITION_RANGE joint=Fl_roll_joint actual_rad=1.0000" in reasons[3]


@pytest.mark.parametrize("invalid,detail", [
    ("dimensions", "reason=dimensions positions=11"),
    ("nonfinite", "reason=nonfinite entries=position:3:nan"),
    ("names", "reason=joint_names names=unknown"),
])
def test_rejected_joint_feedback_preserves_specific_reason(invalid, detail):
    node = SimpleNamespace(joints_received=True)
    msg = SimpleNamespace(position=[0.0] * 12, velocity=[0.0] * 12,
                          effort=[0.0] * 12, name=[])
    if invalid == "dimensions":
        msg.position.pop()
    elif invalid == "nonfinite":
        msg.position[3] = float("nan")
    else:
        msg.name = ["unknown"] * 12
    Controller._joint_state_cb(node, msg)
    assert not node.joints_received
    assert node.joint_fault_detail.startswith(detail)


def test_manual_park_log_is_one_short_line_with_previous_state():
    node = SimpleNamespace(state="WALK", get_logger=Mock(), state_lock=threading.Lock(),
                           cmd_vel=np.ones(3), filtered_action=np.ones(12), joint_pos=np.zeros(12),
                           last_safe_park_request=0.0, safe_park_double_press_window=1.0, soft_fault_times={},
                           _sensors_ready=Mock(return_value=True), _trigger_hard_estop=Mock())
    node._log_safety = MethodType(Controller._log_safety, node)
    Controller._request_safe_park(node, 1.0, "code=MANUAL source=xbox_rb")
    assert node.state == "SAFE_PARK"
    node.get_logger.return_value.warn.assert_called_once_with(
        "event=SAFE_PARK state=WALK code=MANUAL source=xbox_rb")
    np.testing.assert_array_equal(node.cmd_vel, 0)
    np.testing.assert_array_equal(node.transition_target_pos, 0)
    Controller._request_safe_park(node, 1.2, "code=MANUAL source=xbox_rb")
    node._trigger_hard_estop.assert_called_once_with("code=PARK_DOUBLE_PRESS interval_s=0.200 limit_s=1.000")
    node.get_logger.return_value.warn.assert_called_once()
