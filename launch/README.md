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
- `with_joy` (default: `false`): Launch ROS 2 `joy_node` for `/dev/input/js0`.
- `with_hardware` (default: `true`): Launch CAN hardware driver.
- `use_cpp_hardware` (default: `true`): Use C++ node (`robstride_can_node`). Set to `false` for Python driver.
- `with_controller` (default: `true`): Launch RL controller node.

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
