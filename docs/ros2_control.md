# Jaguar ros2_control migration

Both zero-reference and nominal launchers on `main` use the ros2_control stack
below. The legacy standalone runtime remains available through its explicit
diagnostic launcher.

```text
Xbox / keyboard -> command mux -> Python Jaguar policy (50 Hz)
                                   |
                          /joint_commands (named JointState)
                                   |
controller_manager (200 Hz read -> update -> write)
  |-- joint_state_broadcaster -> /joint_states -> policy / robot_state_publisher
  |-- jaguar_control/ImpedanceController
  |      position / velocity / kp / kd / effort (12 joints)
  |      jaguar_safety commands and states
  `-- jaguar_control/JaguarSystem -> calibrated RS00 manager -> can0 + can1
```

The standalone `robstride_can_node` is not started by the shared launch. There
is one CAN owner and no separate custom CAN scheduling thread. Existing policy
observations, zero/nominal action references, encoder offsets, joint directions,
CAN IDs and RS00 protocol encoding are retained. Joint feedback now uses the
physical URDF names (`Bl_roll_joint`, etc.); the policy accepts these aliases.
The impedance controller accepts both these names and the legacy hardware
names, checks completeness and uniqueness, then maps to hardware order.

## Pion reference

The requested repository was cloned into `/tmp/pion_ros2_control_plugin`:

```bash
git clone git@github.com:Pion-Robots/pion_ros2_control_plugin.git
git checkout 8b5c8bc2d48be20a4c87dc96f2f78490dbaf16ef
```

Its `cybergear_control` actuator documents the lifecycle and impedance interface
pattern, including Kp/Kd. Its generic actuator has TODOs for motor enable/disable
and does not include Jaguar's two-bus calibration and safety handling. This
migration therefore implements a Jaguar-specific `SystemInterface` around the
existing calibrated transport rather than substituting the generic single-motor
actuator. The Pion repository is a development reference, not an unpinned runtime
dependency. The installed Jaguar package exports its own two plugin classes.

API references: [Jazzy hardware interface](https://control.ros.org/jazzy/doc/api/classhardware__interface_1_1SystemInterface.html),
[Jazzy controller interface](https://control.ros.org/jazzy/doc/api/classcontroller__interface_1_1ControllerInterface.html),
and [Pion source](https://github.com/Pion-Robots/pion_ros2_control_plugin/tree/8b5c8bc2d48be20a4c87dc96f2f78490dbaf16ef/cybergear_ros2_control/cybergear_control).

## Build and launch

Install Jazzy dependencies first:

```bash
sudo apt-get install ros-jazzy-ros2-control ros-jazzy-ros2-controllers
cd /home/qr1/sim2real
source scripts/build.sh
```

Start the zero-reference model after a successful build:

```bash
sudo ./scripts/bringup_canbus.sh
ros2 launch jaguar_control sim2real_zero.launch.py
```

The launch publishes a robot description with the ros2_control resources,
starts `ros2_control_node`, and spawns `joint_state_broadcaster` plus
`jaguar_impedance_controller` as an activation group. Hardware activation enables
the motors with zero gains/torque. The Python policy starts in passive STANDBY;
standing/walking still require explicit commands.

To inspect the running stack from another terminal:

```bash
source scripts/source_ros.sh
source install/setup.bash
export ROS_DOMAIN_ID=1
ros2 control list_controllers
ros2 control list_hardware_components
ros2 control list_hardware_interfaces
```

Use the same domain as the launch. `use_cpp_hardware` and
`torque_safety_enabled` are no longer supported configuration switches on this
branch. The legacy standalone launch remains available explicitly for diagnostics;
never run it alongside the ros2_control hardware on the same CAN buses.

## Offline mock check

This command opens no CAN interfaces and launches no IMU, gamepad reader, or RL
policy. Mock hardware only echoes bounded joint commands; it is not a simulator.

```bash
ros2 launch jaguar_control sim2real_zero.launch.py \
  mock_hardware:=true with_controller:=false with_imu:=false with_teleop:=false \
  domain_id:=71
```

Inspect from a terminal using `ROS_DOMAIN_ID=71`:

```bash
ros2 control list_controllers
ros2 topic echo /joint_states --once
ros2 topic echo /jaguar/hardware_status --once
```

Expected controllers are active and status is `PASSIVE_ZERO_TORQUE`. A missing
command stream is permitted while passive. A stale **active** command latches
E-stop after `command_timeout` (0.10 s). Header timestamps do not control this
watchdog; receipt time uses the monotonic clock.

## Safety behavior

| Cause | Action on this branch |
| --- | --- |
| Measured torque above diagnostic threshold | Throttled warning only, including during SAFE_PARK; no state transition |
| Command torque/feedforward above configured cap | Clipped by the hardware manager; firmware torque limit configured to at most 14 Nm |
| Motor error/fault bits; encoder jump; CAN send failure | Latched hardware E-stop |
| Invalid dimensions, duplicate/unknown joints, NaN/Inf, negative Kp/Kd | Latched E-stop; rejected command does not reach the motor |
| Active command stream older than 0.10 s | Latched E-stop from the impedance controller |
| Stale/invalid sensor feedback during active operation | E-stop when feedback cannot support a park trajectory |
| Persistent tracking error or modest position-limit violation | Safe park; a violation that persists/worsens while parking escalates to E-stop |
| Extreme physical position violation | Immediate hardware E-stop |
| Excessive tilt, invalid policy output, severe policy compute lag | Existing policy safe-shutdown handling; park when feedback permits, otherwise E-stop |
| Manual RB / safe-stop topic | Safe park; repeated request within configured window escalates to E-stop |
| LB / Back / emergency-stop topic | Latched E-stop |
| Park acknowledgment timeout, park timeout or failed final park position | E-stop |
| Impedance controller deactivation | Zero command gains/effort and request hardware stop |
| Hardware deactivation, cleanup, shutdown or lifecycle error | Disable all motors and close CAN through the existing manager |

The command cap does not prove physical torque cannot overshoot; measured torque
remains visible. A firmware motor fault still stops operation even if its root
cause relates to current or torque. We do not suppress motor fault bits.

This is a ros2_control migration, not a claim of hard-real-time certification.
The reused transport/safety manager still has locks and allocations, and a total
host/control-loop stall cannot be handled by a watchdog on that same stopped
thread. Controller manager scheduling, firmware behavior and physical stopping
must be validated on the robot separately.

## Recovery

An E-stop remains latched when its input returns false. Reset explicitly:

```bash
ros2 service call /jaguar/reset_fault std_srvs/srv/Trigger '{}'
ros2 topic echo /jaguar/hardware_status
# Wait for PASSIVE_ZERO_TORQUE, then reset the policy controller:
ros2 service call /jaguar/reset_controller std_srvs/srv/Trigger '{}'
```

The service queues a reset, rather than claiming it has completed. Hardware
recovery requires fresh fault-free motor feedback; the policy reset requires
valid sensor feedback. Pre-reset commands are discarded. No reset automatically
stands up or resumes WALK. The mock path can test transport/lifecycle behavior
without pretending to validate actual motor recovery.

## Configuration

| Location | Parameters | Meaning |
| --- | --- | --- |
| Launch | `mock_hardware=false` | Explicit offline transport selection; actual CAN by default |
| Launch | `with_hardware`, `startup_clear_faults` | Enable ros2_control stack and select activation fault clearing |
| Launch | `policy_path`, `policy_reference`, `config_file`, `domain_id`, `with_controller`, `with_imu`, `with_teleop`, `with_joy` | Existing model and node selection |
| `controller_manager.ros__parameters` | `update_rate=200`, `thread_priority=80` | Manager control-loop frequency and scheduling priority |
| `jaguar_impedance_controller.ros__parameters` | `command_timeout=0.10` | Monotonic active-command watchdog, seconds |
| `jaguar_hardware.ros__parameters` | `control_torque_limit_nm=14.0`, optional `motor_params_file` | Hardware parameters embedded in the generated robot description |
| `joint_state_broadcaster.ros__parameters` | `joints`, `interfaces` | Physical named feedback on `/joint_states` |
| `nxp_jaguar_controller.ros__parameters` | `overtorque_threshold=14.0`, gains, ramp, IMU, park settings | Policy configuration; overtorque threshold controls diagnostics only |

The retired standalone `robstride_can_hardware` / `jaguar_can_hardware` sections
are used only by explicit standalone tools. Set the new `jaguar_hardware` section
to change the active ros2_control torque cap. Restart after changing YAML. See
[zero-reference parameter tables](zero_reference.md) for policy tuning.

## Command contract and tests

`/joint_commands` retains the policy's current `sensor_msgs/JointState` encoding:
12 positions, 0 or 12 velocities, and 24 effort entries (`Kp[12], Kd[12]`) or 36
entries (append feedforward torque `[12]`). Names may be physical or legacy
aliases; unnamed commands use the hardware BL/BR/FL/FR order. The old
`/mit_controller/commands` raw-array bypass is not exposed by this controller.

`/joint_states`, `/jaguar/hardware_status`, `/jaguar/motor_diagnostics`,
`/jaguar/control_torque_limit_nm`, `/jaguar/hardware_safe_park`,
`/jaguar/safe_park_active`, `/jaguar/emergency_stop` and `/jaguar/reset_fault`
keep their existing roles. Diagnostics publish through realtime publisher
workers, keeping DDS publication off the manager's update callback.

```bash
source scripts/source_ros.sh
source install/setup.bash
colcon test --base-paths . serial_imu --packages-select jaguar_control serial_imu
colcon test-result --verbose
```

Offline tests cover reference modes, generated URDF/interface consistency,
legacy safety/protocol behavior, torque diagnostics without stopping, and mock
hardware lifecycle, command caps and latched stop/reset handling. None of these
tests establishes successful live-hardware locomotion.
