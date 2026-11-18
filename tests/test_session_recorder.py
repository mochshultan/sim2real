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
    assert rows[-1]['type'] == 'summary'
    assert rows[-1]['dropped_samples'] == 0
    assert rows[-1]['written_records'] == 100
    assert rows[-1]['wall_time_ns'] >= rows[0]['wall_time_ns']
    assert first.path != second.path


def test_current_sessions_have_separate_run_folders(tmp_path):
    recorder = SessionRecorder(tmp_path, {'policy_reference': 'current'})
    assert recorder.close()
    assert recorder.path.name == 'telemetry.jsonl'
    assert recorder.path.parent.parent == tmp_path
    metadata = read_log(recorder.path)[0]
    assert metadata['run_id'] == recorder.path.parent.name
    assert metadata['schema_version'] == 2
    assert metadata['started_at_utc'].endswith('+00:00')


def test_pion_session_records_actual_estimator_and_actor_tensors(tmp_path):
    from control_msgs.msg import DynamicJointState, InterfaceValue
    import rclpy
    import torch
    from pion_controller import PionControllerNode

    root = Path(__file__).parents[1]
    policy = root / 'models/policy_pion_current_height20_swing_native_14999_20261118.pt'
    rclpy.init(args=['--ros-args', '--params-file', str(root / 'config/sim2real_pion.yaml'),
                     '-p', f'policy_path:={policy}', '-p', 'policy_reference:=current',
                     '-p', 'history_initialization:=repeat', '-p', f'session_log_dir:={tmp_path}'])
    node = None
    try:
        node = PionControllerNode()
        for publisher in ('joint_cmd_pub', 'safe_park_active_pub', 'status_pub', 'debug_pub'):
            setattr(node, publisher, Mock())
        path = node.session_recorder.path
        node._control_loop()
        node.imu_received = node.joints_received = True
        node.last_imu_time = node.last_joints_time = time.monotonic()
        node.joint_pos = node.stand_joint_pos.copy()
        node.state = 'STAND_HOLD'
        node._control_loop()
        node.state = 'WALK'
        node.last_imu_time = node.last_joints_time = time.monotonic()
        # Deliberately shuffled physical names and one missing CAN timestamp.
        # Delivery of a new DDS packet must not hide an old CAN reception.
        timing = DynamicJointState()
        timing.joint_names = list(reversed(node.joint_names))
        timing.header.stamp.sec = 123
        stamps = [time.monotonic() - 0.030 - index * 0.001 for index in range(12)]
        stamps[0] = 0.0
        timing.interface_values = [
            InterfaceValue(interface_names=['can_rx_time_s'], values=[stamps[node.joint_names.index(name)]])
            for name in timing.joint_names
        ]
        node._can_feedback_cb(timing)
        node._control_loop()
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
    samples = [row for row in read_log(path) if row['type'] == 'sample']
    assert len(samples) == 3
    assert samples[0]['estimator_input'] is None
    assert samples[1]['actor_output'] is None
    walk = samples[2]
    assert samples[0]['can_feedback_age_s'] == [None] * 12
    assert walk['can_feedback_age_s'][0] is None
    assert walk['can_timing_source_ros_time_ns'] == 123000000000
    for index in range(1, 12):
        assert walk['can_rx_time_monotonic_s'][index] == stamps[index]
        assert walk['can_feedback_age_s'][index] >= 0.030 + index * 0.001
    assert walk['can_timing_message_age_s'] < min(walk['can_feedback_age_s'][1:])
    history = torch.tensor([walk['policy_input']])
    network = node.policy.policy
    expected_packing = torch.cat([history[:, :, a:b].flatten(1)
                                 for a, b in ((0, 3), (3, 6), (6, 9), (9, 21), (21, 33), (33, 45))], 1)
    with torch.no_grad():
        velocity = network.estimator(expected_packing)
        actor_input = (torch.cat((history[:, -1], velocity), 1) - network.mean) / (network.std + network.eps)
        np.testing.assert_array_equal(walk['estimator_input'], expected_packing[0].numpy())
        np.testing.assert_array_equal(walk['estimator_output'], velocity[0].numpy())
        np.testing.assert_array_equal(walk['actor_input'], actor_input[0].numpy())
        np.testing.assert_array_equal(walk['actor_output'], network.actor(actor_input)[0].numpy())
        np.testing.assert_array_equal(walk['policy_output'], node.policy(history)[0].numpy())
    assert walk['policy_action'] == walk['policy_output']
    assert walk['inference_end_wall_time_ns'] >= walk['inference_start_wall_time_ns'] >= walk['wall_time_ns']


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
