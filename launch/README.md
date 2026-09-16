# Launch Files (`launch/`)

ROS 2 launch files for robot bringup, simulation, sensors, and visualization.

## 1. Control and Sim-to-Real

| Launch File | Description | Command |
| :--- | :--- | :--- |
| [`sim2real.launch.py`](./sim2real.launch.py) | Starts the serial IMU driver, RobStride C++ CAN node (200 Hz), and NXP Jaguar RL controller (50 Hz). | `ros2 launch jaguar_control sim2real.launch.py` |
| [`sim2sim_mujoco.launch.py`](./sim2sim_mujoco.launch.py) | Starts the MuJoCo simulation with keyboard teleoperation. | `ros2 launch jaguar_control sim2sim_mujoco.launch.py terrain:=rough` |

### Arguments for `sim2real.launch.py`

- `policy_path` (default: `models/policy.pt`): Path to TorchScript policy model.
- `with_imu` (default: `true`): Launch serial IMU node.
- `with_teleop` (default: `true`): Launch the unified teleop hub. Direct Xbox input works from launch; keyboard input needs a separate TTY.
- `with_joy` (default: `false`): Launch optional ROS 2 `joy_node`, remapped to `/joy_raw` for the teleop hub.
- `with_hardware` (default: `true`): Launch CAN hardware driver.
- `use_cpp_hardware` (default: `true`): Use C++ node (`robstride_can_node`). Set to `false` for Python driver.
- `with_controller` (default: `true`): Launch RL controller node.
- `startup_clear_faults` (default: `true`): With C++ hardware, clear latched motor faults once before startup enable. Set `startup_clear_faults:=false` to disable it; runtime faults still require manual reset.

### Recovering a C++ CAN driver fault

The driver logs `FEEDBACK_FAULT` with the raw motor error bits, encoder positions,
allowed jump, feedback interval, velocity, torque, and temperature. A type-21
report is logged as `MOTOR_FAULT` with its raw fault/warning words. Keep the robot
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
is available only with `use_cpp_hardware:=true`; it has not been validated on a
physical robot.

## 2. Sensors and Perception

| Launch File | Description |
| :--- | :--- |
| [`fast_lio.launch`](./fast_lio.launch) | Starts FAST-LIO LiDAR-inertial odometry, point cloud filter, and elevation mapping. |
| [`livox.launch`](./livox.launch) | Starts the Livox LiDAR driver. |
| [`msg_MID360.launch`](./msg_MID360.launch) | Starts Livox MID-360 LiDAR driver with custom packet format. |
| [`gnss.launch`](./gnss.launch) | Starts RTK-GNSS serial receiver node. |
| [`simple_grid.launch`](./simple_grid.launch) | Starts elevation grid mapping. |

## 3. Visualization and Logging

| Launch File | Description |
| :--- | :--- |
| [`display.launch`](./display.launch) | Starts `robot_state_publisher` and RViz for URDF model inspection. |
| [`display_livox.launch`](./display_livox.launch) | Opens RViz with Livox point cloud displays. |
| [`rosbag_recorder.launch`](./rosbag_recorder.launch) | Records topics (`/joint_states`, `/Imu_data`, `/cmd_vel`, `/livox/lidar`). |
| [`rosbag_display.launch`](./rosbag_display.launch) | Replays recorded rosbags in RViz. |
