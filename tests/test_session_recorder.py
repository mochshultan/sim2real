"""Check persisted telemetry against actual inference and published commands offline."""
import json
from pathlib import Path
import sys
import threading
import time
from unittest.mock import Mock

import numpy as np

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
from session_recorder import SessionRecorder


def read_log(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def test_recorder_drains_and_keeps_sessions_separate(tmp_path):
    first = SessionRecorder(tmp_path, {'joint_names': ['joint']})
    second = SessionRecorder(tmp_path, {})
    for i in range(100):
        first.record({'type': 'sample', 'sequence': i, 'position': [i] * 12})
    assert first.close()
    assert first.close()
    assert second.close()
    rows = read_log(first.path)
    assert rows[0]['joint_names'] == ['joint']
    assert [row['sequence'] for row in rows[1:-1]] == list(range(100))
    assert rows[-1] == {'type': 'summary', 'dropped_samples': 0}
    assert first.path != second.path


def test_slow_storage_drops_without_blocking_control_thread(tmp_path):
    recorder = SessionRecorder(tmp_path, {}, queue_size=1)
    started, release = threading.Event(), threading.Event()
    original = recorder._stream

    class SlowStream:
        def write(self, data):
            started.set()
            release.wait(2)
            return original.write(data)
        flush = original.flush
        close = original.close

    recorder._stream = SlowStream()
    try:
        recorder.record({'type': 'sample'})
        assert started.wait(1)
        recorder.record({'type': 'sample'})
        for _ in range(20):
            recorder.record({'type': 'sample'})
        assert recorder.dropped == 20
    finally:
        release.set()
        assert recorder.close()
    assert read_log(recorder.path)[-1]['dropped_samples'] == 20


def test_writer_failure_is_reported_without_raising_in_control_thread(tmp_path):
    recorder = SessionRecorder(tmp_path, {})
    recorder.record({'unserializable': object()})
    recorder._thread.join(timeout=2)
    assert recorder.error
    recorder.record({'type': 'sample'})
    assert recorder.dropped == 1
    assert not recorder.close()


def test_zero_controller_records_exact_policy_input_and_clamped_motor_output(tmp_path, monkeypatch):
    import rclpy
    import torch
    from nxp_jaguar_controller import NXPJaguarControllerNode, ISAAC_LIMITS_LOWER, ISAAC_LIMITS_UPPER

    # No hardware node or executor; publishers are mocked before any control cycle.
    monkeypatch.setenv('ROS_DOMAIN_ID', '179')
    policy = Path(__file__).parents[1] / 'models/policy_adaboot_l2c2_zero_reference_clearance5cm_19999_20261111.pt'
    rclpy.init(args=['--ros-args', '-p', 'policy_reference:=zero', '-p', f'policy_path:={policy}',
                     '-p', f'session_log_dir:={tmp_path}', '-p', 'history_initialization:=repeat'])
    node = None
    try:
        node = NXPJaguarControllerNode()
        node.joint_cmd_pub = Mock()
        node.safe_park_active_pub = Mock()
        node.status_pub = Mock()
        node.debug_pub = Mock()
        path = node.session_recorder.path
        node._control_loop()  # Sensor-wait cycle must still be recorded.
        node.imu_received = node.joints_received = True
        node.last_imu_time = node.last_joints_time = time.monotonic()
        node.joint_pos = node.stand_joint_pos.copy()
        node.state = 'STAND_HOLD'
        node._control_loop()
        node.state = 'WALK'
        node.walk_start_time = node.get_clock().now().nanoseconds / 1e9 - 2
        node.last_imu_time = node.last_joints_time = time.monotonic()
        node.policy = Mock(return_value=torch.tensor([[10.] * 12]))
        node._control_loop()
        sent = node.joint_cmd_pub.publish.call_args.args[0]
        node._trigger_hard_estop('code=MANUAL')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
    rows = read_log(path)
    assert rows[0]['policy_reference'] == 'zero'
    samples = [row for row in rows if row['type'] == 'sample']
    assert len(samples) == 3
    assert samples[0]['motor_output'] is None
    assert not samples[0]['joints_valid']
    assert samples[1]['policy_action'] is None
    assert samples[1]['state'] == 'STAND_HOLD'
    walk = samples[2]
    assert walk['policy_evaluated']
    np.testing.assert_array_equal(walk['observation'], node.policy.call_args.args[0][0, -1].numpy())
    np.testing.assert_array_equal(walk['policy_history'], node.policy.call_args.args[0][0].numpy())
    assert walk['policy_action'] == [10.] * 12
    np.testing.assert_allclose(walk['motor_output']['position_rad'],
                               np.clip([2.5] * 12, ISAAC_LIMITS_LOWER, ISAAC_LIMITS_UPPER))
    assert walk['motor_output']['position_rad'] == list(sent.position)
    assert walk['motor_output']['kp'] == list(sent.effort[:12])
    assert rows[-2]['type'] == 'estop'
    assert rows[-2]['motor_output']['kp'] == [0.] * 12
