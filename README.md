# NXP Jaguar Quadruped: Sim-to-Real Deployment (Branch: `main`)

Reinforcement learning control deployment from Isaac Lab (AdaBoot-Ideal) to RobStride RS00 actuators via ROS 2.

The robot runtime targets **Ubuntu 24.04 LTS / ROS 2 Jazzy / Python 3.12**.
For an existing Ubuntu 22.04/Humble installation, follow the
[Jazzy migration guide](docs/upgrade_jazzy.md) before rebuilding.

### Current-position deployment

`sim2real` (`bash scripts/run.sh`) defaults to current-position actions with the
48D actor. Both presets use native Pion joint order, repeat-initialized history,
Kp45/Kd4.5, action clip10 and `q_target = q_measured + 0.25 * action` at50Hz.

| Preset | Actor input | Default model | Launch file |
|---|---|---|---|
| `sim2real` or `sim2real --48d` |45D observation +3D estimated velocity | QR1 plane orient9999 resume (`20261120b`) | `sim2real_48d.launch.py` |
| `sim2real --64d` |45D observation +3D velocity +16D context | PPO VHIP mean-context9999 | `sim2real_64d.launch.py` |
| MoE (ROS launch directly) |45D observation +3D velocity +16D context | Measured-URDF MoE-Loco L2C2_4999, four experts | `sim2real_moe.launch.py` |

All presets accept history `[B,5,45]` and return12 actions. The selected actor
dimension is checked at controller startup. `sim2real.launch.py` also defaults
to the48D current-position profile. `sim2real_curpos.launch.py` remains the64D
preset for existing commands and simulation. To change models, select the
matching dimension launcher and pass an explicit `policy_path:=...`.

The48D checkpoint/export contract is recorded in
`models/deployment_pion_qr1_plane_orient_9999_20261120b.json`; the64D contract is in
`models/deployment_pion_qr1_dreamwaq_l2c2_height19_native_9999_20261120.json`.
Training configuration snapshots were not supplied for these checkpoints, so
full training configuration parity and physical motion remain unverified.

The archived nominal Jaguar29996 profile is available via `sim2real --old`.
That option selects its legacy controller, configuration and nominal action
reference. Additional ROS launch arguments are forwarded to all presets.

Choose one launcher per controller session:

```bash
# Default current-position48D
bash scripts/run.sh
# Explicit48D /64D current-position launchers
ros2 launch jaguar_control sim2real_48d.launch.py
ros2 launch jaguar_control sim2real_64d.launch.py

# Pion QR1 DreamWaQ L2C2 height19 checkpoint9999
./scripts/sim2real_curpos
# Same controller with a Pion MuJoCo plant, browser visualization, no CAN
./scripts/sim2sim_curpos
```

`sim2real_curpos.launch.py` replaces the former `sim2real_zero.launch.py` entrypoint.
It selects `pion_controller.py`, `config/sim2real_pion.yaml`,
`policy_pion_qr1_dreamwaq_l2c2_height19_native_9999_20261120.pt`, and `policy_reference=current`,
and repeat-initialized five-frame history. Position observations remain calibrated
`q` relative to0rad. Actions now use **q_target=q_measured+0.25*clipped_action**,
anchored once per50Hz policy step and held by the200Hz motor loop. They do not
integrate from the preceding target. Standing uses collar0, hip-1.1 and knee+1.1rad on all four legs.

DreamWaQ CENet normalizes history225 and estimates velocity3 plus context16.
The actor receives observation45 + velocity3 + context16 =64D. The complete
native export uses the deterministic context mean; supplied actor-only exports
require augmented64D input and are not the curpos default.

The native export and dedicated controller use FL/FR/BL/BR within each
joint group, following the established Pion deployment convention. The export applies no joint permutation and clips actions
to +/-10. Named messages map the native Pion joints to the CAN hardware. The original actor checkpoint and export
matched exactly for zero/random/stress histories and batches1/16/64.

Both current-position presets use the driver/firmware14Nm ceiling.
Motor calibration remains shared with the physical hardware. The current-position launcher requires an export carrying
the matching action-reference contract, so an old absolute-zero policy cannot
silently be used with a measured-position action anchor. Earlier zero policies
remain archived under their versioned filenames.

Sim2sim runs **the same `pion_controller.py` ROS node**, including history,
inference, current-position targets, standing/walk ramp, mechanical target limits,
and controller safety checks. The MuJoCo node only publishes calibrated joint/IMU
feedback and consumes `/joint_commands`; it does not contain a policy. It uses
DDS domain71 by default, separate from hardware domain1, and starts no CAN or
serial-IMU nodes. Viser serves `http://<host-ip>:8080`; its sliders send velocity
commands to the real controller. Simulation auto-arms from the standing spawn,
then enters policy mode through the controller's normal A/B inputs. Physics is
paused until the first active impedance command.

See [the current-position guide](docs/current_position.md) for setup, headless
runs, model provenance and interpretation limits. Real hardware motion with this
new checkpoint has not been tested. The 9999 checkpoint did not include training
configuration snapshots; its runtime profile follows the existing Pion deployment.
Export hashes, checkpoint contracts and verification limits are recorded
in `models/deployment_pion_qr1_dreamwaq_l2c2_height19_native_9999_20261120.json`.

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
ros2 launch jaguar_control sim2real_curpos.launch.py \
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


### Pion MoE-Loco launcher

```bash
colcon build --packages-select jaguar_control --symlink-install
source install/setup.bash
ros2 launch jaguar_control sim2real_moe.launch.py
```

The dedicated launcher selects the complete native MoE/CENet export in
`models/history/moeloco_measured_l2c2_5k/`. It includes the current-position
controller settings and accepts the same launch arguments. The existing PPO
launcher/default model is unchanged. The selected MoE was trained with the
measured Pion URDF, VHIP rewards, L2C2, oracle-centred KL, context mean and
AdaBoot max increase0.01: 2000 oracle +3000 adaptation updates, checkpoint4999.

Controller input remains `[B,5,45]`, actor/router input64, output12 native Pion
actions. The export includes velocity/context estimation, four action experts
and their softmax router, with no privileged contact input, critic or LSTM.
The paired manifest records the checkpoint, saved config, asset provenance,
hashes and offline validation; hardware motion is not validated here.

To export another compatible checkpoint from this repository:

```bash
python scripts/export_moeloco_policy.py --checkpoint /path/model_4999.pt --output /path/native_moe.pt
```

The standalone exporter does not overwrite files. A deployed versioned policy
also needs its matching `deployment_*.json` manifest, as checked by the controller.
