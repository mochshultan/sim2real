# Scripts (`scripts/`)

Control nodes, teleoperation tools, calibration utilities, diagnostics, and bringup scripts for the NXP Jaguar quadruped.

## 1. Control and Hardware Nodes

| Script | Type | Description |
| :--- | :---: | :--- |
| [`nxp_jaguar_controller.py`](./nxp_jaguar_controller.py) | ROS 2 Node | Evaluates TorchScript policy at 50 Hz from the 48-D observation vector, handles state transitions (`STANDBY`, `STANDUP`, `STAND_HOLD`, `WALK`, `SITDOWN`, `SAFE_SHUTDOWN`), and enforces tilt and torque limits. |
| [`can_hardware_node.py`](./can_hardware_node.py) | ROS 2 Node | Python SocketCAN driver fallback. Publishes `/joint_states` at 200 Hz. |
| [`parameters.py`](./parameters.py) | Module | CAN IDs, motor directions, calibration offsets, joint names, and nominal standing angles. |
| [`robstride_motor_lib.py`](./robstride_motor_lib.py) | Module | SocketCAN communication library for RobStride RS actuators with MIT impedance control (`p_des`, `v_des`, `kp`, `kd`, `tau_ff`). |

## 2. Teleoperation

| Script | Type | Description |
| :--- | :---: | :--- |
| [`keyboard_teleop.py`](./keyboard_teleop.py) | ROS 2 Node | Teleoperation interface for keyboard input, Bluetooth or USB gamepads, and ROS 2 `/joy` topics. |
| [`gamepad_reader.py`](./gamepad_reader.py) | Tool / Module | Non-blocking Linux gamepad reader for `/dev/input/js*`. Maps Bluetooth Xbox (15 buttons) and USB xpad (11 buttons). |
| [`remote_xbox_forwarder.py`](./remote_xbox_forwarder.py) | Standalone | UDP forwarder for Xbox controllers connected to an operator laptop. |

## 3. Hardware Calibration and Diagnostics

| Script | Type | Description |
| :--- | :---: | :--- |
| [`calibrate_sit_zero.py`](./calibrate_sit_zero.py) | Interactive Tool | Calibrates hip and knee angles in steps of 0.05 rad and saves offsets with `[S]`. |
| [`calibrate_stand_pose.py`](./calibrate_stand_pose.py) | Interactive Tool | Tunes standing joint angles in steps of 0.05 rad and prints posture values on exit with `[S]`. |
| [`test_sit_stand.py`](./test_sit_stand.py) | Interactive Tool | Tests sit (0 rad) and stand transitions with S-curve cosine trajectories via keyboard or gamepad. |
| [`check_states.py`](./check_states.py) | ROS 2 Node | Terminal dashboard displaying the 48-D observation vector, sensor rates, and joint order mappings. |
| [`check_joints.py`](./check_joints.py) | Standalone Tool | Reads motor encoder angles over CAN with zero control gains ($K_p=0, K_d=0$). |
| [`set_robostride_zero.py`](./set_robostride_zero.py) | Standalone Tool | Writes the current position as mechanical zero into RobStride RS00 motor EEPROM. |
| [`scan_robostride_ids.py`](./scan_robostride_ids.py) | Standalone Tool | Scans `can0` and `can1` for connected motor node IDs and factory IDs. |
| [`robstride_motor_test.py`](./robstride_motor_test.py) | Standalone Tool | Individual motor tests for position mode, torque mode, ID assignment, and passive reading. |

## 4. Shell Bringup and Utilities

| Script | Description |
| :--- | :--- |
| [`bringup_canbus.sh`](./bringup_canbus.sh) | Sets up `can0` and `can1` SocketCAN interfaces at 1 Mbps with queue length 1000. |
| [`bringup_imu.sh`](./bringup_imu.sh) | Sets `/dev/ttyUSB0` permissions and starts the `serial_imu talker` node. |
| [`plot_torques.sh`](./plot_torques.sh) | Opens `rqt_plot` with 12 RobStride motor channels. |
| [`stop_all.sh`](./stop_all.sh) | Stops robot processes, ROS 2 daemons, and CAN interfaces. |

## 5. Sensor Transforms and Math

| Script | Description |
| :--- | :--- |
| [`livox_cloud_transform.py`](./livox_cloud_transform.py) | Converts Livox PointCloud messages to `sensor_msgs/PointCloud2`. |
| [`livox_imu_transform.py`](./livox_imu_transform.py) | Applies pitch and roll mounting corrections to Livox IMU data. |
| [`livox_odom_transform.py`](./livox_odom_transform.py) | Publishes static transform from `/odom` to `/camera_init`. |
| [`jaguar_utils.py`](./jaguar_utils.py) | Utilities for URDF parsing, TorchScript policy loading, and interpolation. |
| [`isaacgym_torch_utils.py`](./isaacgym_torch_utils.py) | Quaternion and vector math routines. |
| [`legged_gym_math.py`](./legged_gym_math.py) | Yaw transformations and angle wrapping routines. |
