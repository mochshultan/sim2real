# 🚀 ROS 2 & Sensor Launch Files (`launch/`)

This directory contains ROS 2 launch files for system bringup, sim-to-real deployment, sim-to-sim validation, sensor integration (Livox LiDAR, IMU, GNSS), and RViz visualization.

---

## 📑 Launch Files Reference

### 🐾 1. Robot Control & Sim-to-Real
| Launch File | Description | Usage Example |
| :--- | :--- | :--- |
| [`sim2real.launch.py`](./sim2real.launch.py) | **Primary One-Click Sim-to-Real Bringup**. Launches serial IMU driver, RobStride C++ CAN hardware node (200 Hz), and NXP Jaguar RL controller node (50 Hz). | `ros2 launch jaguar_control sim2real.launch.py` |
| [`sim2sim_mujoco.launch.py`](./sim2sim_mujoco.launch.py) | Launches the interactive MuJoCo Sim-to-Sim simulator with real-time camera tracking and keyboard teleoperation. | `ros2 launch jaguar_control sim2sim_mujoco.launch.py terrain:=rough` |

#### `sim2real.launch.py` Arguments:
- `policy_path` (default: `models/policy.pt`): Absolute path to TorchScript policy model.
- `with_imu` (default: `true`): Launch serial IMU node.
- `with_joy` (default: `false`): Launch standard ROS 2 `joy_node` for `/dev/input/js0`.
- `with_hardware` (default: `true`): Launch CAN hardware driver.
- `use_cpp_hardware` (default: `true`): Use hard real-time C++ node (`robstride_can_node`) instead of Python driver.
- `with_controller` (default: `true`): Launch RL controller node.

---

### 📡 2. Perception & Sensor Integration
| Launch File | Description |
| :--- | :--- |
| [`fast_lio.launch`](./fast_lio.launch) | Launches FAST-LIO LiDAR-inertial odometry, PointCloud transformations, self-filter, and elevation mapping. |
| [`livox.launch`](./livox.launch) | Launches the Livox LiDAR ROS driver. |
| [`msg_MID360.launch`](./msg_MID360.launch) | Launches Livox MID-360 LiDAR driver with custom packet format. |
| [`gnss.launch`](./gnss.launch) | Launches RTK-GNSS serial receiver node. |
| [`simple_grid.launch`](./simple_grid.launch) | Launches elevation grid mapping pipeline. |

---

### 🖥️ 3. Visualization & Logging
| Launch File | Description |
| :--- | :--- |
| [`display.launch`](./display.launch) | Launches `robot_state_publisher` and RViz for URDF model inspection. |
| [`display_livox.launch`](./display_livox.launch) | Launches RViz configured with Livox point cloud displays. |
| [`rosbag_recorder.launch`](./rosbag_recorder.launch) | Records essential robot topics (`/joint_states`, `/Imu_data`, `/cmd_vel`, `/livox/lidar`). |
| [`rosbag_display.launch`](./rosbag_display.launch) | Visualizes recorded rosbags in RViz. |
