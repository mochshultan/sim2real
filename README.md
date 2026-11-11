# NXP Jaguar Quadruped: Sim-to-Real Deployment (Branch: `main`)

Reinforcement learning control deployment from Isaac Lab (AdaBoot-Ideal) to RobStride RS00 actuators via ROS 2.

The robot runtime targets **Ubuntu 24.04 LTS / ROS 2 Jazzy / Python 3.12**.
For an existing Ubuntu 22.04/Humble installation, follow the
[Jazzy migration guide](docs/upgrade_jazzy.md) before rebuilding.

### Current policy

The default deployment artifact is deterministically fixed at
`models/policy.pt`; `policy_path` can override it explicitly. It is the
estimator-only TorchScript export of Jaguar AdaBoot Ideal L2C2 checkpoint
iteration 29996 from the Isaac Lab run
`nxp_jaguar_baseline_tibia_adaboot_ideal_l2c2/2026-10-22_01-40-39_stand021-kp45kd45-stand10pct-cmd5s-fourfeet-coxa-clear8cm-ori002-torque95of145-w001-resume19997-10kit`.
This continues checkpoint 19997 for another 10k iterations.

The source checkpoint is
`models/model_adaboot_ideal_l2c2_kp45kd45_clear8cm_29996_20261022.pt`.
Its runtime actor is `models/policy.pt`. The identical named export
`models/policy_adaboot_ideal_l2c2_kp45kd45_clear8cm_29996_20261022.pt`
is retained as its identity pair. Both files must keep the same SHA-256.
The checkpoint SHA-256 is
`4544c25e7602a294b8ed6df58e98532c3a1a08b066741b4eab56e386b5e56311`;
the policy SHA-256 is
`15c2e2cf7775361b7edbebe3e2181f0e4bae5ad8024c826b340cffb891bef5a0`.
CPU comparisons against both checkpoint reconstruction and the original run
export matched exactly for batches 1, 16, and 64 of five 45-D frames.
This was offline validation; no live hardware motion has been tested.

Normal and transition gains are Kp 45 / Kd 4.5. The nominal reference remains
Jaguar's: front hip/knee -1.51/+1.22 rad, rear -1.49/+1.20 rad, rolls 0 rad.
Observations contain `q - q0`; actions use `q_des = q0 + 0.25 * action`.
Joint groups retain FR/FL/BR/BL order, without a Pion joint-order adapter.
The Pion policy and deployment artifacts previously added here were removed.

### Selecting either Jaguar policy

Choose one launcher per controller session:

```bash
# Model 29996: observations q - q0, targets q0 + 0.25 * action
ros2 launch jaguar_control sim2real.launch.py

# Model 5999: observations q, targets 0.25 * action
ros2 launch jaguar_control sim2real_zero.launch.py
```

The zero-reference launcher selects
`models/policy_adaboot_ideal_l2c2_zero_reference_5999_20261103.pt` and sets
`policy_reference:=zero` in the shared controller. Its training run is
`nxp_jaguar_adaboot_ideal_l2c2_zero_reference/2026-11-03_21-57-17_jaguar-kp45kd45-zero-scale025-ema0002-6144env-fresh6k-20261103T183206`.
The export matches both checkpoint 5999 and the original training export exactly
on CPU for batches 1, 16, and 64. Artifact hashes and training parameters are saved
in `models/deployment_adaboot_ideal_l2c2_zero_reference_5999_20261103.json` and the
matching model directory. This policy has not been tested on live hardware.

Both launchers use the same standing pose, encoder calibration, gains, and safety
handling. Zero reference applies to policy observations and action targets;
it does not command all joints to zero during standing. Stop the current launch
before selecting the other model. Optional arguments such as
`with_hardware:=false` also apply to the zero-reference launcher.
See the [zero-reference launch and configuration tables](docs/zero_reference.md)
for all flags, YAML parameters, defaults, and tuning examples.

For model 29996, clearance target is 8 cm, base height target 21 cm, and orientation weight -0.02.
The training soft torque penalty starts at 13.775 Nm (95% of 14.5 Nm), with
weight -0.01. Stand commands are sampled at 10%, resampled every 5 seconds.
The exact contract, hashes, and validation are recorded in
`models/deployment_adaboot_ideal_l2c2_kp45kd45_clear8cm_29996_20261022.json`;
training snapshots are in `models/adaboot_ideal_l2c2_kp45kd45_clear8cm_29996_20261022/`.

The export accepts `(batch, 5, 45)` and returns `(batch, 12)`. Its velocity
estimator is `225 -> 128 -> 64 -> 3`; the actor is
`48 -> 512 -> 256 -> 128 -> 12`, with embedded observation normalization.

## Overview

The standing target is coxa `0.0` rad on all legs, front hip/knee `-1.59/+1.40` rad,
and rear hip/knee `-1.59/+1.38` rad, defined by `STAND_ANGLE` in
`scripts/parameters.py`. The controller uses it for
`STANDUP` and `STAND_HOLD`, with bounded IMU leveling trim when enabled. Walking
uses the selected policy's reference and blends from the standing pose during
the walk ramp: nominal for model 29996, zero for model 5999.

This branch runs the 12 calibrated RobStride RS00 actuators through `jaguar_control/JaguarSystem` under `controller_manager` at 200 Hz. The impedance controller claims position, velocity, effort, Kp and Kd command interfaces. The Python policy remains at 50 Hz. See the [ros2_control migration and mock validation guide](docs/ros2_control.md).

### DDS buffer errors and unexpected motor stop

Repeated `sequence size exceeds remaining buffer` from several unrelated nodes
can indicate incompatible DDS discovery or message traffic. Check that every
machine/container on the robot's domain uses the same ROS distribution and
matching message definitions. Humble/Jazzy/Rolling discovery incompatibilities
are tracked in [rmw_fastrtps #797](https://github.com/ros2/rmw_fastrtps/issues/797).
The error alone does not identify why the motors stopped: look for
`event=ESTOP` or `event=SAFE_PARK` and the accompanying `code=...` in the full log.

`sim2real.launch.py` now defaults to DDS domain **1** when `ROS_DOMAIN_ID` is
unset; it preserves an explicitly exported domain. To isolate from other ROS
installations on domain 0, stop the old launch and use an unused domain:

```bash
export ROS_DOMAIN_ID=1
ros2 launch jaguar_control sim2real.launch.py domain_id:=1
```

Export the same `ROS_DOMAIN_ID` in every keyboard, remote joystick, RViz and
diagnostic terminal before starting those programs. Other ROS distributions
and unrelated robots should use a different domain. If CLI discovery looks
stale, run `ros2 daemon stop` in that terminal before retrying. For a robot
whose ROS nodes all run on one computer, `export ROS_LOCALHOST_ONLY=1` before
launch additionally restricts discovery to that computer; this prevents remote
ROS teleop/RViz connections.

DDS feedback publication runs outside the CAN thread, in a separate executor
callback group, so a blocked feedback publisher does not block CAN cycles or
command callbacks. The 100 ms command watchdog and stale-feedback E-stop remain
enabled. Communication loss can still trigger a motor stop; do not increase
watchdog limits to hide DDS errors. Validate with the robot supported before
trying WALK again.

## Runtime and configuration

```text
Xbox -> keyboard_teleop -> /cmd_vel/xbox ----\
                                             mux -> /cmd_vel -> RL controller
Keyboard -> keyboard_cmd_vel -> /cmd_vel/keyboard -/                 |
IMU + /joint_states ---------------------------------------------->|
                                                                  v
                                                        /joint_commands
                                                                  |
                                                        ros2_control / JaguarSystem
                                                                  |
                                                               CAN motors
```

The serial IMU publishes `/Imu_data`; the controller also accepts `/imu/data`.
The driver returns `/joint_states`. Mode buttons are separate `/joy` pulses.
`/joint_commands` uses `JointState.effort` for 12 Kp followed by 12 Kd values,
not measured torque.

| Component | Configuration / assets |
| --- | --- |
| Controller, Xbox, keyboard and mux | [`config/sim2real.yaml`](config/sim2real.yaml) |
| CAN packet encoding / RS00 ranges | [`config/cybergear_motor_params.yaml`](config/cybergear_motor_params.yaml) |
| Python motor mapping, offsets and standing pose | [`scripts/parameters.py`](scripts/parameters.py) |
| C++ motor mapping and offsets | `setupDefaultJointConfigs()` in [`robstride_hardware_manager.hpp`](include/jaguar_control/robstride_hardware_manager.hpp) |
| Policy observation scales/history | [`scripts/policy_observation.py`](scripts/policy_observation.py), pinned to the training contract |
| Xbox factory calibration | [`config/gamepad_calibration.json`](config/gamepad_calibration.json) |
| Xbox user calibration | `${XDG_CONFIG_HOME:-~/.config}/jaguar_control/gamepad_calibration.json` |
| Robot display | [`urdf/nxp_jaguar.urdf`](urdf/nxp_jaguar.urdf), `meshes/`, `config/jaguar_display.rviz` |

The complete tree, module responsibilities and configuration precedence are in
[the repository map](docs/repository_structure.md). Python and C++ motor maps
remain explicit in their respective languages; update both when recalibrating.
Their offset conventions differ in sign and are checked by regression tests.

## Build and offline verification

Use the system Python with ROS 2 Jazzy, NumPy, PyYAML, pytest and CPU PyTorch.
PyTorch is an additional Python dependency; see [the environment notes](docs/upgrade_jazzy.md).

```bash
source scripts/build.sh
colcon test --base-paths . serial_imu --packages-select jaguar_control serial_imu
colcon test-result --verbose
```

The CMake test suite registers CAN protocol, motion safety, serial parser and
all Python tests. Tests do not open CAN buses or command motors. You can also
run Python tests directly after sourcing Jazzy:

```bash
python3 -m pytest tests -q
```

Runtime installation includes the default policy, its named identity pair and deployment manifest.
The matching source checkpoint and training YAML snapshots stay
in source for reproducibility. Old model variants have been removed.
`build/`, `install/`, `log/`, virtual environments and Python caches are local
outputs excluded from Git.

## Bringup

Support the robot and verify the physical stop before powering actuators.
Configure CAN interfaces, then start the stack:

```bash
sudo ./scripts/bringup_canbus.sh
source scripts/source_ros.sh
source install/setup.bash
ros2 launch jaguar_control sim2real.launch.py
```

The launch starts IMU, C++ hardware, Xbox input, mux and controller. The
controller starts in passive `STANDBY`; launch does not auto-stand or auto-walk.
The shared launch uses ros2_control on this branch. Set `mock_hardware:=true` for an offline resource check; the legacy standalone launch is separate.
See [launch arguments and fault recovery](launch/README.md).

## Xbox and keyboard

Xbox has priority while its heartbeat is fresh (default 0.30 s). Left stick
controls X/Y and right stick controls yaw. Xbox acceleration is limited to
1 m/s² and 1 rad/s² by default. Deceleration follows the input immediately,
including stick release to zero, on the next mux cycle (20 ms). Reversal stops
the old direction immediately and accelerates the new direction from zero.
The planar acceleration limit applies to the combined X/Y change.

The combined X/Y speed is limited to **0.7 m/s** with RT released. RT raises
that limit proportionally: `0.7 + 0.3 * RT` m/s, up to **1.0 m/s** at full
press. Yaw stays limited to **1.0 rad/s**. Releasing the stick returns its
command to zero immediately on the next input/mux cycle.

| Xbox control | Action |
| --- | --- |
| Left stick Y / X | Forward/backward / lateral movement |
| Right stick X | Yaw |
| Start / Menu | From STAND_HOLD: enable policy WALK; from WALK: stop policy and smoothly return to STAND_HOLD |
| A | Stand up |
| X | Sit down |
| Y | Zero velocity command |
| RT | Proportional linear speed boost, 0.7–1.0 m/s |
| RB | Safe park |
| LB / Back | Hard E-stop |
| B, LT, D-pad, LS/RS click, Xbox/Guide, right stick Y | Unassigned |

Start acts once per press. Entering WALK still requires valid feedback and
upright IMU orientation. Returning from WALK uses the configured stand
transition duration; policy inference stops immediately.

Start terminal keyboard control separately:

```bash
ros2 run jaguar_control keyboard_cmd_vel.py
```

Hold W/A/S/D and Q/E; P/O adjusts the target. SPACE requests safe park.
The keyboard source refuses input while a local joystick is connected.
See [the script guide](scripts/README.md) for mode controls and utilities.

Calibrate Xbox without editing installed package files:

```bash
ros2 run jaguar_control calibrate_xbox.py
```

The generated user profile overrides the factory JSON after input-node restart.

## Safety and recovery

Command torque is capped at 14 Nm in the hardware/firmware configuration.
Measured overtorque is diagnostic only and never requests SAFE_PARK or E-stop,
including during parking. `overtorque_threshold` and five-cycle debounce control
warnings only. Clipping a command does not prove that physical torque cannot
overshoot. Firmware-reported motor faults still latch an E-stop.

Motor/CAN faults, invalid or stale feedback, encoder jumps, invalid/stale active
commands, manual E-stop, and failed parking retain stop handling. Persistent
position/tracking violations, excessive tilt and severe policy timing faults
request safe park when feedback permits; otherwise they stop immediately.
Recovery remains manual. See [the safety matrix and recovery commands](docs/ros2_control.md).

## Safety logs

Safety events use short `key=value` fields. `event` identifies the action and
`code` identifies the cause. Joint, sensor, measured value, threshold and timing
are included where applicable.

```text
event=WARNING state=WALK code=OVERTORQUE joint=Fr_roll_joint actual_nm=14.100 limit_nm=14.000 samples=5 required_samples=5 action=LOG_ONLY
[driver] time_ms=1793850000000 event=ESTOP code=FEEDBACK_TIMEOUT joint=BL_collar_joint bus=can1 can_id=4 age_s=0.260000 limit_s=0.250000
```

Codes include `TRACKING`, `POSITION_LIMIT`, `POSITION_RANGE`, `SENSOR_INVALID`,
`SENSOR_TIMEOUT`, `FEEDBACK_INVALID`, `FEEDBACK_TIMEOUT`, `MOTOR_FAULT`,
`MOTOR_ERROR`, `ENCODER_JUMP`, `COMMAND_TIMEOUT`, `CAN_WRITE`, `TILT`,
`POLICY_OUTPUT`, `COMPUTE_LAG`, `PARK_ACK_TIMEOUT`, `PARK_TIMEOUT`, `PARK_POSITION`
and `MANUAL`. Several simultaneous joint violations are logged separately.
Repeated fault events are suppressed within a stop/park episode; a new cause,
joint, sensor or motor fault bit pattern is still recorded after E-stop.
Log-only overtorque warnings remain throttled to once per second. Existing
thresholds, debounce rules and stop actions are unchanged by the log formatting.

Capture both ROS logs and the C++ driver's stderr in one file:

```bash
set -o pipefail
ros2 launch jaguar_control sim2real_zero.launch.py \
  2>&1 | tee "sim2real_$(date +%Y%m%d_%H%M%S).log"
```

Firmware torque saturation itself is not reported as a fault by this code.
Motor fault bits are recorded as reported; they are not decoded into assumed
physical causes.

## Calibration, diagnostics and offline tools

Calibration and direct motor tools are manual entry points and are not started
by normal bringup. Their roles and commands are listed in
[`scripts/README.md`](scripts/README.md).

- [PACE collection](docs/pace_system_identification.md) and
  [validation](docs/pace_calibration_validation.md) retain their offline tools
  and guarded active chirp workflow.
- `export_adaboot_policy.py` and `export_adaptive_policy.py` require explicit
  model input paths. Replacing the default policy requires `--set-default`.
- `record_sim2real_benchmark.py` and `monitor_joints.py` provide manual telemetry.
- [CAN driver provenance and IMU details](docs/cybergear_driver_and_imu.md).
- [Training kinematics/reward notes](docs/training_notes.md).

This repository deploys ROS 2 control. ROS 1 nodes/launches and the incomplete
MuJoCo launch have been removed; simulation belongs to the training workspace.
