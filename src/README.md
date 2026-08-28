# 🛠️ C++ Driver Source Files (`src/`)

This directory contains the ROS 2 node implementation for low-level hard real-time hardware execution.

---

## 📄 Source Files

### `robstride_can_node.cpp`
The primary hard real-time ROS 2 driver node (`robstride_can_hardware`).

- **Real-Time Scheduling**: Runs a dedicated POSIX thread configured with `SCHED_FIFO` (Priority 80) and `clock_nanosleep(CLOCK_MONOTONIC)` for deterministic 200 Hz execution.
- **Published Topics**:
  - `/joint_states` (`sensor_msgs/msg/JointState` @ 200 Hz): Actual angular positions, velocities, and torques.
  - `/jaguar/motor_diagnostics` (`std_msgs/msg/Float32MultiArray` @ 1 Hz): Motor temperatures.
  - `/jaguar/hardware_status` (`std_msgs/msg/String` @ 1 Hz): Operational state and watchdog overruns.
- **Subscribed Topics**:
  - `/joint_commands` (`sensor_msgs/msg/JointState`): Desired target joint angles and custom PD gains.
  - `/mit_controller/commands` (`std_msgs/msg/Float64MultiArray`): Direct MIT impedance control array.
  - `/jaguar/emergency_stop` (`std_msgs/msg/Bool`): Hardware E-stop trigger.

---

## 🚀 Execution
```bash
ros2 run jaguar_control robstride_can_node
```
