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

`jaguar_unified_teleop.ros__parameters.linear_speed_base` sets the Xbox planar
speed with RT released (0.7 m/s). `linear_speed_boost` sets the full-RT speed
(1.0 m/s). Intermediate RT positions interpolate between these values. The
combined X/Y magnitude follows this envelope, including diagonal input. The
shared per-axis limits remain hard caps; `max_wz` stays 1.0 rad/s.

Launch passes `config_file` to nodes. Explicit ROS parameter overrides take
precedence over defaults. Standalone Python tools load the packaged default
YAML through `project_paths.py`; installed scripts resolve package share.
Restart nodes after editing configuration.

Xbox acceleration uses the combined X/Y change and an independent yaw limit.
Deceleration is immediate on each axis, including stick release. Reversing an
axis stops the previous direction and accelerates from zero. Keyboard release,
source timeout and safety stops can also zero commands immediately. The
benchmark override is speed-clamped by the controller and bypasses teleop ramps.

## ros2_control and torque diagnostics

- `controller_manager`: 200 Hz read/update/write and SCHED_FIFO priority 80.
- `jaguar_hardware`: hardware torque cap (14 Nm); optional `motor_params_file`.
- `jaguar_impedance_controller`: active command timeout, 0.10 s.
- `joint_state_broadcaster`: publishes named calibrated feedback on `/joint_states`.
- `robstride_can_hardware` and `jaguar_can_hardware`: retained for standalone tools only.

`nxp_jaguar_controller.overtorque_threshold` controls feedback warnings only.
Overtorque never requests park or E-stop on this branch; the old
`torque_safety_enabled` flag/parameter has been removed. Motor fault bits and all
other safety checks remain active. See [the migration guide](../docs/ros2_control.md).
