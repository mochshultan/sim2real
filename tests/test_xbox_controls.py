"""Xbox input regression tests without opening a joystick or commanding motors."""
import sys
import threading
from pathlib import Path
from unittest.mock import Mock

import pytest

sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))
from gamepad_reader import (LinuxGamepadReader, XboxState, linear_speed_limit,
                            update_gamepad_state)


def make_reader():
    reader = LinuxGamepadReader.__new__(LinuxGamepadReader)
    reader.state = XboxState()
    reader.prev_state = XboxState()
    reader.lock = threading.RLock()
    reader.max_vx = reader.max_vy = reader.max_wz = 1.0
    reader.linear_speed_base = 0.7
    reader.linear_speed_boost = 1.0
    reader.deadzone = 0.1
    reader.calibration = None
    reader.callback = Mock()
    return reader


@pytest.mark.parametrize("bluetooth,lt_axis,rt_axis", [(True, 5, 4), (False, 2, 5)])
@pytest.mark.parametrize("pressed", ["lt", "rt"])
def test_trigger_identity(bluetooth, lt_axis, rt_axis, pressed):
    reader = make_reader()
    axes = [0.0] * 8
    axes[lt_axis] = axes[rt_axis] = -1.0
    axes[lt_axis if pressed == "lt" else rt_axis] = 1.0
    reader._update_state_from_raw(axes, [0] * 15, bluetooth)
    assert reader.state.lt == (1.0 if pressed == "lt" else 0.0)
    assert reader.state.rt == (1.0 if pressed == "rt" else 0.0)


def make_teleop():
    """Run the production callbacks with fake ROS publishers and no hardware."""
    import ast
    import time
    from types import SimpleNamespace
    tree = ast.parse((Path(__file__).parents[1] / "scripts/keyboard_teleop.py").read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "UnifiedTeleopNode")
    names = {"_on_gamepad_event", "_publish_loop", "_ros_joy_cb", "_status_cb"}
    cls.bases = []
    cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
    def twist():
        return SimpleNamespace(linear=SimpleNamespace(), angular=SimpleNamespace())
    namespace = dict(XboxState=XboxState, Dict=dict, Any=object, Bool=SimpleNamespace,
                     String=object, Joy=lambda: SimpleNamespace(header=SimpleNamespace()),
                     Twist=twist, time=time, update_gamepad_state=update_gamepad_state,
                     linear_speed_limit=linear_speed_limit, DEADZONE=0.1)
    namespace.update({name: "" for name in ("C_CYAN", "C_GREEN", "C_RESET", "C_YELLOW", "C_RED")})
    exec(compile(ast.Module(body=[cls], type_ignores=[]), "<teleop callbacks>", "exec"), namespace)
    node = namespace["UnifiedTeleopNode"]()
    node.lock = threading.RLock()
    node.input_source = "gamepad"
    node.xbox_active = False
    node.max_vx = node.max_vy = node.max_wz = 1.0
    node.linear_speed_base = 0.7
    node.linear_speed_boost = 1.0
    node.previous_ros_buttons = {"remote": [], "raw": []}
    node.vx = node.vy = node.wz = 0.0
    node.btn_standup_pulse = node.btn_walk_pulse = node.btn_sit_pulse = False
    node.btn_policy_toggle_pulse = False
    node.gamepad_reader = Mock()
    node.gamepad_reader.is_connected.return_value = True
    node.cmd_vel_pub = Mock()
    node.joy_pub = Mock()
    node.estop_pub = Mock()
    node.safe_stop_pub = Mock()
    node.get_clock = Mock()
    return node


def test_teleop_start_publishes_one_toggle_and_b_is_unassigned():
    node = make_teleop()
    state = XboxState()
    state.connected = True
    node._on_gamepad_event(state, {"btn_start": True})
    node._publish_loop()
    assert node.joy_pub.publish.call_args.args[0].buttons == [0] * 7 + [1]
    node._publish_loop()
    assert node.joy_pub.publish.call_args.args[0].buttons == [0] * 8
    node._on_gamepad_event(state, {"btn_b": True})
    node._publish_loop()
    assert node.joy_pub.publish.call_args.args[0].buttons == [0] * 8


@pytest.mark.parametrize("button", ["btn_lb", "btn_back", "btn_rb"])
def test_teleop_safety_clears_simultaneous_policy_request(button):
    node = make_teleop()
    state = XboxState()
    state.connected = True
    node._on_gamepad_event(state, {"btn_start": True, button: True})
    node._publish_loop()
    assert node.joy_pub.publish.call_args.args[0].buttons == [0] * 8
    assert node.estop_pub.publish.called or node.safe_stop_pub.publish.called


@pytest.mark.parametrize("rt,expected", [(0.0, 0.7), (0.25, 0.775), (0.5, 0.85), (1.0, 1.0)])
def test_rt_proportionally_raises_speed_without_changing_yaw(rt, expected):
    state = XboxState()
    state.ly = 1.0
    state.rx = -1.0
    state.rt = rt
    state.compute_velocities()
    assert state.vx == pytest.approx(expected)
    assert state.vy == 0.0
    assert state.wz == 1.0
    # LT has no speed effect.
    state.lt = 1.0
    state.compute_velocities()
    assert state.vx == pytest.approx(expected)


@pytest.mark.parametrize("rt", [0.0, 0.5, 1.0])
@pytest.mark.parametrize("x,y", [(1, 1), (-1, 1), (1, -1), (-1, -1)])
def test_diagonal_speed_stays_inside_planar_envelope(rt, x, y):
    import math
    state = XboxState()
    state.lx, state.ly, state.rt = x, y, rt
    state.compute_velocities()
    assert math.hypot(state.vx, state.vy) == pytest.approx(0.7 + 0.3 * rt)
    assert math.hypot(state.vx, state.vy) <= 0.7 + 0.3 * rt + 1e-15


def test_release_returns_zero_and_rt_release_restores_base_cap():
    state = XboxState()
    state.ly = state.rt = 1.0
    state.compute_velocities()
    assert state.vx == 1.0
    state.rt = 0.0
    state.compute_velocities()
    assert state.vx == 0.7
    state.ly = 0.0
    state.compute_velocities()
    assert (state.vx, state.vy, state.wz) == (0.0, 0.0, 0.0)


@pytest.mark.parametrize("rt,expected", [(-1, 0.7), (2, 1.0), (float("nan"), 0.7), (float("inf"), 0.7)])
def test_invalid_or_out_of_range_trigger_cannot_exceed_cap(rt, expected):
    assert linear_speed_limit(rt) == expected


@pytest.mark.parametrize("base,boost", [(0, 1), (1, 0.7), (0.7, float("nan")), (-1, 1)])
def test_invalid_speed_configuration_is_rejected(base, boost):
    with pytest.raises(ValueError):
        linear_speed_limit(0, base, boost)


@pytest.mark.parametrize("bluetooth,lt_axis,rt_axis,start_button", [(True, 5, 4, 11), (False, 2, 5, 7)])
def test_ros_joy_matches_direct_input_and_start_edges(bluetooth, lt_axis, rt_axis, start_button):
    from types import SimpleNamespace
    reader = make_reader()
    raw_axes = [0.0] * 8
    raw_axes[lt_axis] = -1.0
    raw_axes[rt_axis] = 0.0  # Half RT -> 0.85 m/s.
    raw_axes[1] = -1.0
    raw_buttons = [0] * (15 if bluetooth else 11)
    raw_buttons[start_button] = 1
    reader._update_state_from_raw(raw_axes, raw_buttons, bluetooth)
    node = make_teleop()
    msg = SimpleNamespace(axes=[-axis for axis in raw_axes], buttons=raw_buttons)
    node._ros_joy_cb(msg)
    assert (node.vx, node.vy, node.wz) == pytest.approx((reader.state.vx, reader.state.vy, reader.state.wz))
    assert node.vx == pytest.approx(0.85)
    assert node.input_source == "remote"
    node._publish_loop()
    assert node.joy_pub.publish.call_args.args[0].buttons[7] == 1
    node._ros_joy_cb(msg)
    node._publish_loop()
    assert node.joy_pub.publish.call_args.args[0].buttons[7] == 0


def test_ros_safety_buttons_use_the_same_mapping_as_direct_input():
    from types import SimpleNamespace
    for buttons, safety_index in [(15, 6), (11, 4), (15, 7), (11, 5)]:
        node = make_teleop()
        pressed = [0] * buttons
        pressed[safety_index] = 1
        node._ros_joy_cb(SimpleNamespace(axes=[], buttons=pressed))
        assert node.estop_pub.publish.called or node.safe_stop_pub.publish.called
