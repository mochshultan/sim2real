"""Xbox input regression tests without opening a joystick or commanding motors."""
import sys
import threading
from pathlib import Path
from unittest.mock import Mock

import pytest

sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))
from gamepad_reader import LinuxGamepadReader, XboxState


def make_reader():
    reader = LinuxGamepadReader.__new__(LinuxGamepadReader)
    reader.state = XboxState()
    reader.prev_state = XboxState()
    reader.lock = threading.RLock()
    reader.max_vx = reader.max_vy = reader.max_wz = 1.0
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
                     Twist=twist, time=time)
    namespace.update({name: "" for name in ("C_CYAN", "C_GREEN", "C_RESET", "C_YELLOW", "C_RED")})
    exec(compile(ast.Module(body=[cls], type_ignores=[]), "<teleop callbacks>", "exec"), namespace)
    node = namespace["UnifiedTeleopNode"]()
    node.lock = threading.RLock()
    node.input_source = "gamepad"
    node.xbox_active = False
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
