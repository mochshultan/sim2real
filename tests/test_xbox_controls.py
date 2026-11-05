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
