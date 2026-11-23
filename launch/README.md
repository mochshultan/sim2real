# Launch Files (`launch/`)

ROS 2 launch files for robot bringup, simulation, sensors, and visualization.

## 1. Control and Sim-to-Real

| Launch File | Description | Command |
| :--- | :--- | :--- |
| [`sim2real.launch.py`](./sim2real.launch.py) | Default current-position48D profile and shared hardware stack. | `ros2 launch jaguar_control sim2real.launch.py` |
| [`sim2real_48d.launch.py`](./sim2real_48d.launch.py) | Current-position48D, plane orient9999 resume20261120b. | `ros2 launch jaguar_control sim2real_48d.launch.py` |
| [`sim2real_64d.launch.py`](./sim2real_64d.launch.py) | Current-position64D, complete DreamWaQ L2C2 height19_9999. | `ros2 launch jaguar_control sim2real_64d.launch.py` |
| [`sim2real_curpos.launch.py`](./sim2real_curpos.launch.py) | Existing alias for the64D preset. | `ros2 launch jaguar_control sim2real_curpos.launch.py` |

Both presets enforce their actor dimension and select current-position actions
with repeated history. The policy interface remains `[B,5,45]` in both cases.
Sim2sim uses the64D preset with a Pion MuJoCo bridge and no CAN/serial IMU:

```bash
ros2 launch jaguar_control sim2sim_curpos.launch.py
```

See [current-position deployment](../docs/current_position.md) for the action
contract, browser controls and headless verification. The former zero launcher
has been replaced; old exports remain archived.

### Arguments for `sim2real.launch.py`

- `policy_reference` (default: `current`): Joint reference for observations and action targets. The curpos launcher selects `current`; positions are observed relative to0rad and actions anchor to measured q.
- `policy_path` (default: `models/policy_pion_qr1_plane_orient_9999_20261120b.pt`): Path to TorchScript policy model.
- `history_initialization` (default: `repeat`): History cold start; the curpos launcher uses `repeat` for Pion Current10k.
- `actor_input_dim` (default: `48`): Expected internal actor dimension; the64D preset selects64.
- `with_imu` (default: `true`): Launch serial IMU node.
- `with_teleop` (default: `true`): Launch the Xbox input source. Xbox has priority while its heartbeat is active.
- `with_joy` (default: `false`): Launch optional ROS 2 `joy_node`, remapped to `/joy_raw` for the teleop hub.
- `with_hardware` (default: `true`): Launch CAN hardware driver.
- `mock_hardware` (default: `false`): Select offline mock hardware without opening CAN buses. The shared launch uses ros2_control on this branch.
- `with_controller` (default: `true`): Launch RL controller node.
- `startup_clear_faults` (default: `true`): With C++ hardware, clear latched motor faults once before startup enable. Set `startup_clear_faults:=false` to disable it; runtime faults still require manual reset.

The velocity path is:

```text
/cmd_vel/xbox ---- priority 100 ----\
                                      > /cmd_vel -> RL controller
/cmd_vel/keyboard - priority 10 ----/
```

Typical startup:

```bash
ros2 launch jaguar_control sim2real.launch.py
```

The mux keeps Xbox selected while the controller source is alive. The launch
file intentionally does not start keyboard control, so controller and hardware
logs remain readable. To enable keyboard commands, start the keyboard node in a
separate terminal:

```bash
ros2 run jaguar_control keyboard_cmd_vel.py
```

The node reads key-down/key-up events from `/dev/input/event*`, so combinations
such as `W+D` and `W+Q` work while the keys are held. The ROS user must have read
permission for those devices (normally through membership in the system `input`
group).

### Recovering a C++ CAN driver fault

The driver logs `event=ESTOP` with `code=MOTOR_ERROR` or `code=ENCODER_JUMP`,
including raw error bits, encoder positions, allowed jump, interval, velocity,
torque and temperature. A type-21 report uses `code=MOTOR_FAULT` with raw
fault/warning words. Keep the robot
supported and remove the underlying cause before resetting; fault clearing is
not a substitute for inspecting a jam, hot motor, low supply voltage, or bad encoder.

At a fresh `sim2real` launch, the C++ driver performs fault clearing automatically
while motors are disabled. The controller still starts in passive `STANDBY`; it
will not auto-stand or auto-walk. If a fault remains during startup, the driver
stops initialization instead of continuing to enable motors.

After the controller has entered `DISABLED`, request a **manual** reset:

```bash
ros2 service call /jaguar/reset_fault std_srvs/srv/Trigger '{}'
ros2 topic echo /jaguar/hardware_status
ros2 service call /jaguar/reset_controller std_srvs/srv/Trigger '{}'
```

The service acknowledges that reset was *queued*, not that it succeeded. The
hardware status moves through `RESETTING_FAULT` to `PASSIVE_ZERO_TORQUE` only
after all 12 motors provide fresh, fault-free, in-range feedback. On failure it
stays `EMERGENCY_STOPPED`; inspect the log and do not keep retrying a persistent
fault. Call `/jaguar/reset_controller` only after status becomes
`PASSIVE_ZERO_TORQUE`; it requires fresh, in-range IMU and joint feedback and
returns the controller to passive `STANDBY`, not walking. The hardware reset path
is provided by the impedance controller and JaguarSystem on this branch. Hardware motion still requires validation. See [the ros2_control guide](../docs/ros2_control.md).

## 2. Standalone driver and display

| Launch | Purpose | Configuration |
| --- | --- | --- |
| `robstride_standalone.launch.py` | Start only the C++ driver, with startup fault clearing disabled by default | `config_file` defaults to `sim2real.yaml`; `motor_params_file` to `cybergear_motor_params.yaml` |
| `display.launch.py` | Start robot_state_publisher and RViz 2 | `model` defaults to `urdf/nxp_jaguar.urdf`; `rviz_config` to `config/jaguar_display.rviz` |

Display launch reads the selected `model` argument at execution time. The URDF
uses `package://jaguar_control/meshes/...` references so the same assets work
from source and package installation. Joint states must come from a driver or
another publisher.

The IMU package also provides `serial_imu/imu_spec_msg.launch.py` for standalone
serial diagnostics. Legacy ROS 1 and incomplete simulator launches were removed.
