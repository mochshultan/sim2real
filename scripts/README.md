# Scripts (`scripts/`)

Control nodes, teleoperation tools, calibration utilities, diagnostics, and bringup scripts for the NXP Jaguar quadruped.

## 1. Control and Hardware Nodes

| Script | Type | Description |
| :--- | :---: | :--- |
| [`nxp_jaguar_controller.py`](./nxp_jaguar_controller.py) | ROS 2 Node | Evaluates the CPU-only TorchScript policy at 50 Hz from the 5-step history of 45-D observations, handles state transitions, and enforces sensor freshness, tilt, torque, and tracking limits. |
| [`can_hardware_node.py`](./can_hardware_node.py) | ROS 2 Node | Python SocketCAN driver fallback. Publishes `/joint_states` at 200 Hz. |
| [`parameters.py`](./parameters.py) | Module | CAN IDs, motor directions, calibration offsets, joint names, and nominal standing angles. |
| [`robstride_motor_lib.py`](./robstride_motor_lib.py) | Module | SocketCAN communication library for RobStride RS actuators with MIT impedance control (`p_des`, `v_des`, `kp`, `kd`, `tau_ff`). |

## 2. Teleoperation

| Script | Type | Description |
| :--- | :---: | :--- |
| [`keyboard_teleop.py`](./keyboard_teleop.py) | ROS 2 Node | Normalizes keyboard, direct Bluetooth/USB gamepad, and ROS 2 `/joy_raw` input. Publishes velocity on `/cmd_vel` and mode pulses on `/joy`. |
| [`keyboard_cmd_vel.py`](./keyboard_cmd_vel.py) | ROS 2 Node | Terminal keyboard controller with a Linux input-event fallback, fast ramp-up, and Xbox-device lockout. |

### Safe keyboard controller

The node is intentionally not started by `sim2real.launch.py`, so launch output
remains dedicated to controller and hardware logs. Start keyboard control
manually in a separate terminal:

```bash
ros2 run jaguar_control keyboard_cmd_vel.py
```

Hold `W/A/S/D` for planar motion and `Q/E` for yaw. In an interactive terminal,
the node reads keys from that terminal, including over SSH. Without a terminal,
it reads Linux key-down/key-up events and can combine held keys. The default target is `0.30 m/s`
and `0.30 rad/s`; `P` increases and `O` decreases both targets. `SPACE`
requests safe park. Terminal motion stops when key repeat ceases for 0.75 s;
Linux input stops each axis on key release. Direct Linux input requires read
access to `/dev/input/event*`.
The node refuses keyboard input while a local `/dev/input/js*` joystick is
connected.
| [`gamepad_reader.py`](./gamepad_reader.py) | Tool / Module | Non-blocking Linux gamepad reader for `/dev/input/js*`. Maps Bluetooth Xbox (15 buttons) and USB xpad (11 buttons). |
| [`remote_xbox_forwarder.py`](./remote_xbox_forwarder.py) | Standalone | UDP forwarder for Xbox controllers connected to an operator laptop. |

## 3. Hardware Calibration and Diagnostics

| Script | Type | Description |
| :--- | :---: | :--- |
| [`calibrate_sit_zero.py`](./calibrate_sit_zero.py) | Interactive Tool | Calibrates hip and knee angles in steps of 0.05 rad and saves offsets with `[S]`. |
| [`calibrate_stand_pose.py`](./calibrate_stand_pose.py) | Interactive Tool | Tunes standing joint angles in steps of 0.05 rad and prints posture values on exit with `[S]`. |
| [`calibrate_parallel_link.py`](./calibrate_parallel_link.py) | Interactive Tool | Holds one suspended leg with low-gain impedance control and independently tests hip motion and the 1:1 knee parallelogram transmission. |
| [`test_sit_stand.py`](./test_sit_stand.py) | Interactive Tool | Tests sit (0 rad) and stand transitions with S-curve cosine trajectories via keyboard or gamepad. |
| [`check_states.py`](./check_states.py) | ROS 2 Node | Terminal dashboard displaying sensor rates and joint order mappings. |
| [`check_joints.py`](./check_joints.py) | Standalone Tool | Reads motor encoder angles over CAN with zero control gains ($K_p=0, K_d=0$). |
| [`set_robostride_zero.py`](./set_robostride_zero.py) | Standalone Tool | Writes the current position as mechanical zero into RobStride RS00 motor EEPROM. |
| [`scan_robostride_ids.py`](./scan_robostride_ids.py) | Standalone Tool | Scans `can0` and `can1` for connected motor node IDs and factory IDs. |
| [`robstride_motor_test.py`](./robstride_motor_test.py) | Standalone Tool | Individual motor tests for position mode, torque mode, ID assignment, and passive reading. |

## 4. Shell Bringup and Utilities

| Script | Description |
| :--- | :--- |
| [`build.sh`](./build.sh) | Builds ROS 2 packages (`jaguar_control`, `serial_imu`) and sources the overlay workspace. |
| [`bringup_canbus.sh`](./bringup_canbus.sh) | Sets up `can0` and `can1` SocketCAN interfaces at 1 Mbps with queue length 1000. |
| [`bringup_imu.sh`](./bringup_imu.sh) | Sets `/dev/ttyUSB0` permissions and starts the `serial_imu talker` node. |
| [`plot_torques.sh`](./plot_torques.sh) | Opens `rqt_plot` with 12 RobStride motor channels. |
| [`stop_all.sh`](./stop_all.sh) | Stops robot processes, ROS 2 daemons, and CAN interfaces. |
| [`wifi_switch.sh`](./wifi_switch.sh) | Saves a static IPv4 configuration for `erc_5G` and switches between `erc_5G` and `ntnu_guest` using NetworkManager. |

### Pindah Wi-Fi dengan IP statis

Jalankan dari folder proyek. Atur alamat sekali sebelum pertama kali menyambung:

```bash
./scripts/wifi_switch.sh setup
./scripts/wifi_switch.sh erc
```

`setup` meminta IP/prefix, gateway, dan DNS lalu menyimpannya di profil
NetworkManager, tanpa berpindah jaringan. Gunakan IP yang dialokasikan untuk
komputer ini pada subnet router `erc_5G`; reservasi atau keluarkan IP tersebut
dari pool DHCP router agar tidak bentrok dengan perangkat lain.

Konfigurasi juga dapat diberikan langsung. Contoh berikut harus disesuaikan
dengan jaringan router:

```bash
./scripts/wifi_switch.sh setup 192.168.1.50/24 192.168.1.1 1.1.1.1,8.8.8.8
```

Untuk kembali dan melihat koneksi:

```bash
./scripts/wifi_switch.sh guest
./scripts/wifi_switch.sh status
```

Perintah `erc` juga menyimpan pilihan untuk boot berikutnya: profil `erc_5G`
tersedia sebelum login, autoconnect aktif dengan prioritas tertinggi, dan
autoconnect profil Wi-Fi lain dimatikan. Jika `erc_5G` tidak terjangkau, komputer
tidak otomatis berpindah ke jaringan lain.
Perintah `guest` mengaktifkan profil `ntnu_guest` yang sudah tersimpan untuk
sementara; pilihan otomatis saat boot tetap `erc_5G`.
Password `erc_5G` memakai kredensial profil yang tersimpan; jika belum tersedia,
`nmcli --ask` memintanya saat menghubungkan. Jika profil `erc_5G` belum ada,
`setup` membuat profil WPA Personal dengan autoconnect dimatikan. Untuk mengubah
alamat, jalankan `setup` kembali. Gunakan `sudo` jika izin NetworkManager ditolak.
Saat dijalankan melalui SSH di Wi-Fi yang diganti, sesi akan terputus; sambungkan
kembali memakai alamat pada jaringan tujuan.

## 5. PACE System Identification Data

| Script | Type | Description |
| :--- | :---: | :--- |
| [`pace/record_jaguar_joint_data.sh`](./pace/record_jaguar_joint_data.sh) | Passive ROS bag recorder | Records joint state/command and hardware context; publishes no commands. |
| [`pace/convert_rosbag_to_pace.py`](./pace/convert_rosbag_to_pace.py) | Offline converter | Reorders named joints, aligns/resamples a bag, and writes PACE tensors; never controls hardware. |
| [`pace/create_pace_default_candidate.py`](./pace/create_pace_default_candidate.py) | Offline utility | Creates the nominal Jaguar actuator candidate for the same-data validation comparison. |
| [`pace/analyze_pace_replay.py`](./pace/analyze_pace_replay.py) | Offline analysis | Reports per-joint position and velocity RMSE from a saved PACE replay. |
| [`pace/run_jaguar_chirp.py`](./pace/run_jaguar_chirp.py) | Guarded active test | Opt-in FR three-joint chirp with separate ascending fit and descending validation profiles, timestamped command CSV, and JSON metadata. Both are limited to +/-0.03 rad on a fully suspended robot with a 3 Nm driver torque cap. |

## 6. Sensor Transforms and Math

| Script | Description |
| :--- | :--- |
| [`livox_cloud_transform.py`](./livox_cloud_transform.py) | Converts Livox PointCloud messages to `sensor_msgs/PointCloud2`. |
| [`livox_imu_transform.py`](./livox_imu_transform.py) | Applies pitch and roll mounting corrections to Livox IMU data. |
| [`livox_odom_transform.py`](./livox_odom_transform.py) | Publishes static transform from `/odom` to `/camera_init`. |
| [`jaguar_utils.py`](./jaguar_utils.py) | Utilities for URDF parsing, TorchScript policy loading, and interpolation. |
| [`isaacgym_torch_utils.py`](./isaacgym_torch_utils.py) | Quaternion and vector math routines. |
| [`legged_gym_math.py`](./legged_gym_math.py) | Yaw transformations and angle wrapping routines. |
