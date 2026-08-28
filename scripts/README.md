# 🚀 Python & Shell Scripts Reference (`scripts/`)

This directory contains control nodes, teleoperation hubs, calibration tools, diagnostics, and bringup shell scripts for the **NXP Jaguar Quadruped**.

---

## 📑 Scripts Directory Index

### 🧠 1. Core Control & Hardware Nodes
| Script | Type | Description |
| :--- | :---: | :--- |
| [`nxp_jaguar_controller.py`](./nxp_jaguar_controller.py) | ROS 2 Node | **High-Level Sim-to-Real RL Controller Node**. Ingests 48-D observation vector, runs TorchScript JIT policy inference at 50 Hz, executes state transitions (`STANDBY`, `STANDUP`, `STAND_HOLD`, `WALK`, `SITDOWN`, `SAFE_SHUTDOWN`), and enforces tilt/overtorque failsafes. |
| [`can_hardware_node.py`](./can_hardware_node.py) | ROS 2 Node | Python-based SocketCAN hardware driver fallback (publishes `/joint_states` @ 200 Hz). |
| [`parameters.py`](./parameters.py) | Module | Central configuration for hardware CAN IDs, motor directions, angular calibration offsets, joint naming, and nominal standing poses. |
| [`robstride_motor_lib.py`](./robstride_motor_lib.py) | Module | Core Python SocketCAN communication library for RobStride RS-series actuators supporting MIT impedance control mode (`p_des`, `v_des`, `kp`, `kd`, `tau_ff`). |

---

### 🕹️ 2. Teleoperation & Remote Control
| Script | Type | Description |
| :--- | :---: | :--- |
| [`keyboard_teleop.py`](./keyboard_teleop.py) | ROS 2 Node | Unified teleoperation hub supporting simultaneous SSH terminal keyboard control, direct Bluetooth/USB Xbox gamepad, and remote ROS 2 `/joy` topics. |
| [`gamepad_reader.py`](./gamepad_reader.py) | Tool / Module | Universal non-blocking Linux Gamepad reader (`/dev/input/js*`). Auto-detects 15-button Bluetooth Xbox and 11-button USB xpad controller mappings. |
| [`remote_xbox_forwarder.py`](./remote_xbox_forwarder.py) | Standalone (Remote PC) | Lightweight UDP telemetry forwarder to run on a remote PC/laptop when the Xbox controller is connected to the operator laptop rather than the robot. |

---

### 🔧 3. Hardware Calibration & Diagnostics
| Script | Type | Description |
| :--- | :---: | :--- |
| [`test_sit_stand.py`](./test_sit_stand.py) | Interactive Tool | Standalone Sit (0.0 rad) and Standup tester using smooth S-curve cosine trajectory interpolation (zero jerk). Supports keyboard and Xbox gamepad. |
| [`check_states.py`](./check_states.py) | ROS 2 Node | Live diagnostic dashboard verifying all 48 dimensions of the Actor observation vector, sensor rates, and joint order mappings against Isaac Lab expectations. |
| [`check_joints.py`](./check_joints.py) | Standalone Tool | Direct CAN passive sensor checker (Kp=0, Kd=0). Reads actual motor encoder angles in real-time with zero applied motor torque. |
| [`set_robostride_zero.py`](./set_robostride_zero.py) | Standalone Tool | Flashes the current physical position as mechanical zero into RobStride RS00 motor EEPROM memory (Type 6 private protocol command). |
| [`scan_robostride_ids.py`](./scan_robostride_ids.py) | Standalone Tool | Scans `can0` and `can1` buses to detect connected RobStride motor CAN node IDs and unique factory IDs. |
| [`robstride_motor_test.py`](./robstride_motor_test.py) | Standalone Tool | Low-level diagnostic script for individual motor testing (position mode, torque mode, ID change, passive sensing). |

---

### 🔌 4. Shell Bringup, Plotting & Teardown Scripts
| Script | Description |
| :--- | :--- |
| [`bringup_canbus.sh`](./bringup_canbus.sh) | Initializes `can0` and `can1` SocketCAN network interfaces at 1 Mbps bitrate (`txqueuelen 1000`). |
| [`bringup_imu.sh`](./bringup_imu.sh) | Validates `/dev/ttyUSB0` serial port permissions and launches the `serial_imu talker` ROS 2 node. |
| [`plot_torques.sh`](./plot_torques.sh) | **1-Click Multi-Motor Telemetry Plotter**. Automatically launches `rqt_plot` configured with all 12 RobStride RS00 motor channels (`effort`, `position`, or `velocity`). |
| [`stop_all.sh`](./stop_all.sh) | Gracefully terminates all background robot processes, stops ROS 2 daemons, and brings down CAN network interfaces. |

---

### 📡 5. Sensor Transforms & Legacy Utilities
| Script | Description |
| :--- | :--- |
| [`livox_cloud_transform.py`](./livox_cloud_transform.py) | Converts Livox custom PointCloud messages to standard `sensor_msgs/PointCloud2`. |
| [`livox_imu_transform.py`](./livox_imu_transform.py) | Applies mounting pitch/roll rotation corrections to Livox LiDAR IMU data. |
| [`livox_odom_transform.py`](./livox_odom_transform.py) | Static transform broadcaster from `/odom` to `/camera_init`. |
| [`jaguar_utils.py`](./jaguar_utils.py) | Utility functions for URDF parsing, TorchScript policy loading, and mathematical interpolations. |
| [`isaacgym_torch_utils.py`](./isaacgym_torch_utils.py) | Quaternion and vector transformation math functions. |
| [`legged_gym_math.py`](./legged_gym_math.py) | Math helper routines for yaw application and angle wrapping. |
