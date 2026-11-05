# Runtime configuration

| File | Reader | Purpose |
| --- | --- | --- |
| `sim2real.yaml` | Main/standalone launch, controller, mux, Xbox, keyboard, hardware defaults | Shared motion limits and per-node runtime parameters |
| `cybergear_motor_params.yaml` | C++ `loadCybergearRS00()` | RS00 packet ranges and actuator encoding |
| `gamepad_calibration.json` | `gamepad_reader.py` | Factory controller calibration; user XDG profile takes precedence |
| `jaguar_display.rviz` | RViz 2 via display launch | Robot/grid display and `/robot_description` subscription |
| `99-usb-can.rules` | udev, when explicitly installed by the operator | Stable USB-CAN device identification |

`sim2real.yaml` is the runtime YAML. `/**.ros__parameters` contains `max_vx`,
`max_vy`, `max_wz`, `linear_accel_limit` and `angular_accel_limit`; all default to
1.0. Controller, Xbox, keyboard and mux declare these same parameters. Per-node
blocks configure the driver, controller, joy source and mux timeouts.

Launch passes `config_file` to nodes. Explicit ROS parameter overrides take
precedence over defaults. Standalone Python tools load the packaged default
YAML through `project_paths.py`; installed scripts resolve package share.
Restart nodes after editing configuration.

Xbox acceleration uses the combined X/Y change and an independent yaw limit.
Deceleration is immediate on each axis, including stick release. Reversing an
axis stops the previous direction and accelerates from zero. Keyboard release,
source timeout and safety stops can also zero commands immediately. The
benchmark override is speed-clamped by the controller and bypasses teleop ramps.

## Overtorque watchdog

Set `nxp_jaguar_controller.ros__parameters.torque_safety_enabled` in
`sim2real.yaml` and restart the controller:

- `true` (default): sustained feedback above `overtorque_threshold` requests
  SAFE_PARK; another overload while parking escalates to E-stop.
- `false`: the same overload only produces a warning, limited to once per
  second, and control continues without a torque-triggered state change.

Override this setting when launching, without editing YAML:

```bash
ros2 launch jaguar_control sim2real.launch.py torque_safety_enabled:=false
ros2 launch jaguar_control sim2real.launch.py torque_safety_enabled:=true
```

The launch flag takes precedence over `config_file`. If omitted, the controller
uses the YAML setting (or its default `true` if the YAML does not specify it).

Both modes keep the existing five-cycle debounce (100 ms at 50 Hz).
This flag controls only the controller's feedback overtorque watchdog.
The low-level/firmware torque caps, motor fault handling, manual E-stop,
and other safety checks remain active; a firmware motor fault can still stop control.
