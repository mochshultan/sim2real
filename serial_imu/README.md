# 🧭 ROS 2 Serial IMU Driver (`serial_imu`)

ROS 2 driver package for the onboard 9-DOF serial IMU sensor (CHAOHE / Hiwonder protocol) used on the **NXP Jaguar Quadruped Robot**.

---

## 📌 Features

- **High-Rate Telemetry**: Streams calibrated 9-DOF IMU data (quaternion orientation, angular velocity, and linear acceleration) at **up to 500 Hz**.
- **Standard ROS 2 Messages**: Publishes `sensor_msgs/msg/Imu` on `/Imu_data` (configurable).
- **Auto-Reconnection**: Non-blocking I/O with automatic port reconnection if the USB cable is power-cycled.
- **Configurable Parameters**: Configurable port path, baud rate, TF frame ID, and topic names.

---

## ⚙️ Parameters

| Parameter | Type | Default | Description |
| :--- | :---: | :---: | :--- |
| `port` | `string` | `"/dev/ttyUSB0"` | Serial port device path |
| `frame_id` | `string` | `"imu_link"` | TF frame ID in message header |
| `topic_name` | `string` | `"/Imu_data"` | Published ROS 2 topic name |

---

## 🚀 Usage

### 1. Build the Package
```bash
cd /home/erc/sim2real
colcon build --base-paths serial_imu --symlink-install
source install/setup.bash
```

### 2. Run the Publisher Node
```bash
# Using default port /dev/ttyUSB0
ros2 run serial_imu talker

# Or with custom port parameter
ros2 run serial_imu talker --ros-args -p port:=/dev/ttyUSB1
```

### 3. Run with Launch File
```bash
ros2 launch serial_imu imu_spec_msg.launch.py port:=/dev/ttyUSB0
```

### 4. Verify Output
```bash
# Echo the IMU topic:
ros2 topic echo /Imu_data

# Or run the listener node:
ros2 run serial_imu listener
```

---

## 📦 Nodes & Executables

- **`talker`** (`src/serial_port.cpp`, `src/ch_serial.c`): Reads serial data packets, decodes checksums, and publishes `sensor_msgs/msg/Imu`.
- **`listener`** (`src/sub_spec.cpp`): Formatted terminal subscriber for debugging and orientation inspection.
