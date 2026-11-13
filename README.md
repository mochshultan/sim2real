# NXP Jaguar Quadruped: Sim-to-Real Deployment (Branch: `main`)

Reinforcement learning control deployment from Isaac Lab (AdaBoot-Ideal) to RobStride RS00 actuators via ROS 2.

The robot runtime targets **Ubuntu 24.04 LTS / ROS 2 Jazzy / Python 3.12**.
For an existing Ubuntu 22.04/Humble installation, follow the
[Jazzy migration guide](docs/upgrade_jazzy.md) before rebuilding.

### Current policy

The normal launcher loads `models/policy.pt`, the estimator-only TorchScript
export of nominal checkpoint **39996**. It continues checkpoint 29996 for
10,000 updates using the original nominal rewards with 5 cm clearance,
15% standing commands, and the linear 32 V RS00 actuator approximation.
The base-height target remains 21 cm with weight -1; the original coxa-standing
and feet-contact-standing rewards remain active.

The source checkpoint is
`models/model_adaboot_l2c2_nominal32v_clearance5cm_39996_20261112.pt`.
The identical named export is
`models/policy_adaboot_l2c2_nominal32v_clearance5cm_39996_20261112.pt`.
Its SHA-256 is `bc37ca24b38f2d659cfcccb9d1101219cdc5dc576003ebcd2dd5a4049d48ddcb`.
The export embeds action clipping at +/-10 before target and previous-action
feedback. CPU comparisons against the native training actor matched exactly
for zero/random/stress histories and batches 1/16/64 (maximum difference 0.0).
The export takes five proprioceptive frames and uses estimated velocity only;
the privileged critic is not deployed.

Hashes, validation, and actuator assumptions are in
`models/deployment_adaboot_l2c2_nominal32v_clearance5cm_39996_20261112.json`;
training snapshots are in the matching model directory. Model 29996 remains
archived under its versioned filename. No live hardware motion was tested.

Normal and transition gains are Kp 45 / Kd 4.5. The nominal reference remains
Jaguar's: front hip/knee -1.51/+1.22 rad, rear -1.49/+1.20 rad, rolls 0 rad.
Observations contain `q - q0`; actions use `q_des = q0 + 0.25 * action`.
Joint groups retain FR/FL/BR/BL order, without a Pion joint-order adapter.
The Pion policy and deployment artifacts previously added here were removed.

### Selecting either Jaguar policy

Use `bash scripts/run.sh` (or the local `sim2real` shortcut) for the current
nominal model 39996. Add `--old` to use the previous nominal model 29996:

```bash
sim2real --old
# Equivalent from the repository:
bash scripts/run.sh --old
```

Additional ROS launch arguments are forwarded, for example
`sim2real --old with_hardware:=false`. Explicit `policy_path:=...`,
`policy_reference:=...`, or `history_initialization:=...` arguments override
the preset. Encoder calibration remains the latest local calibration.

Choose one launcher per controller session:

```bash
# Model 39996: observations q - q0, targets q0 + 0.25 * clipped_action
ros2 launch jaguar_control sim2real.launch.py

# Pion model 11500: observations q, targets 0.25 * clipped_action
ros2 launch jaguar_control sim2real_zero.launch.py
```

The zero-reference launcher selects
`models/policy_pion_adaboot_l2c2_zero_32v_height25_11500_20261113.pt` and sets
`policy_reference:=zero` and `history_initialization:=repeat`. This is the
latest Pion AdaBoot-L2C2 checkpoint 11500, trained with the estimated 32 V RS00
actuator curve, a 25 cm base-height target (weight -10), 5 cm directional
clearance, and 15% standing commands.

Pion training uses FL/FR/BL/BR within each joint group; the controller uses
FR/FL/BR/BL. The export remaps position, velocity and previous-action history
into Pion order before estimation, then maps actions back to controller order.
The body-frame IMU/command fields keep their order. Clipping +/-10 is embedded.
CPU equality with the native Pion actor was verified for batches 1/16/64 and
zero/random/stress histories (maximum absolute difference 0.0).

Hashes, permutations, and training snapshots are in
`models/deployment_pion_adaboot_l2c2_zero_32v_height25_11500_20261113.json` and
its matching model directory. Jaguar zero policies 19999 and 5999 remain
archived. The current main calibration, standing pose, ros2_control stack,
and 14 Nm hardware cap are retained. Physical motion with Pion 11500 has not
been tested.

The zero-reference launcher uses a standing pose of 0 rad coxa, -1.25 rad hip,
and +1.25 rad knee on all four legs. The nominal launcher uses `STAND_ANGLE`.
Both launchers share encoder calibration, gains, and safety handling. Zero reference applies to policy observations and action targets;
it does not command all joints to zero during standing. Stop the current launch
before selecting the other model. Optional arguments such as
`with_hardware:=false` also apply to the zero-reference launcher.
The zero-reference controller automatically saves observation, state, policy
actions and all 12 motor commands/feedback in per-session JSONL files under
`~/.ros/jaguar_control/sessions/`; the full path is printed on startup.
See the [zero-reference launch and configuration tables](docs/zero_reference.md)
for all flags, YAML parameters, defaults, and tuning examples.

For archived model 29996, clearance target is 8 cm, base height target 21 cm, and orientation weight -0.02.
The training soft torque penalty starts at 13.775 Nm (95% of 14.5 Nm), with
weight -0.01. Stand commands are sampled at 10%, resampled every 5 seconds.
The exact contract, hashes, and validation are recorded in
`models/deployment_adaboot_ideal_l2c2_kp45kd45_clear8cm_29996_20261022.json`;
training snapshots are in `models/adaboot_ideal_l2c2_kp45kd45_clear8cm_29996_20261022/`.

The export accepts `(batch, 5, 45)` and returns `(batch, 12)`. Its velocity
estimator is `225 -> 128 -> 64 -> 3`; the actor is
`48 -> 512 -> 256 -> 128 -> 12`, with embedded observation normalization.

## Overview

The standing target is coxa `0.0` rad on all legs, front hip/knee `-1.39/+1.25` rad,
BL hip/knee `-1.74/+1.28` rad, and BR hip/knee `-1.59/+1.23` rad, defined by `STAND_ANGLE` in
`scripts/parameters.py`. The controller uses it for
`STANDUP` and `STAND_HOLD`, with bounded IMU leveling trim when enabled. Walking
uses the selected policy's reference and blends from the standing pose during
the walk ramp: nominal for model 39996, zero for Pion model 11500.

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
