# Serial IMU Driver (`serial_imu`)

ROS 2 driver for the onboard 9-DOF serial IMU (CHAOHE / Hiwonder protocol) on the NXP Jaguar quadruped.

## Features

- Streams calibrated 9-DOF IMU data (quaternion orientation, angular velocity, linear acceleration) up to 500 Hz.
- Publishes `sensor_msgs/msg/Imu` to `/Imu_data`.
- Reconnects to serial port when power cycles.
- Exposes port path, baud rate, TF frame ID, and topic name as ROS parameters.

## Parameters

| Parameter | Type | Default | Description |
| :--- | :---: | :---: | :--- |
| `port` | `string` | `"/dev/ttyUSB0"` | Serial device path |
| `frame_id` | `string` | `"imu_link"` | TF frame ID in message header |
| `topic_name` | `string` | `"/Imu_data"` | Output ROS 2 topic name |

## Usage

### 1. Build
```bash
cd /home/erc/sim2real
colcon build --base-paths serial_imu --symlink-install
source install/setup.bash
```

### 2. Run Publisher
```bash
ros2 run serial_imu talker
ros2 run serial_imu talker --ros-args -p port:=/dev/ttyUSB1
```

### 3. Launch with Parameters
```bash
ros2 launch serial_imu imu_spec_msg.launch.py port:=/dev/ttyUSB0
```

### 4. Inspect Output
```bash
ros2 topic echo /Imu_data
ros2 run serial_imu listener
```

## Nodes

- `talker` (`src/serial_port.cpp`, `src/ch_serial.c`): Reads serial packets, verifies checksums, and publishes `sensor_msgs/msg/Imu`.
- `listener` (`src/sub_spec.cpp`): Terminal listener printing orientation and acceleration.
