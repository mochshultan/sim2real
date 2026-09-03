# C++ Driver Sources (`src/`)

Hardware execution nodes for the NXP Jaguar quadruped.

## `robstride_can_node.cpp`

ROS 2 driver node for RobStride RS00 motor communication over SocketCAN.

- **Scheduling**: Runs a POSIX thread with `SCHED_FIFO` priority 80 and `clock_nanosleep(CLOCK_MONOTONIC)` at 200 Hz.
- **Published Topics**:
  - `/joint_states` (`sensor_msgs/msg/JointState`, 200 Hz): Measured positions, velocities, and torques.
  - `/jaguar/motor_diagnostics` (`std_msgs/msg/Float32MultiArray`, 1 Hz): Motor temperatures.
  - `/jaguar/hardware_status` (`std_msgs/msg/String`, 1 Hz): Bus health and watchdog overruns.
- **Subscribed Topics**:
  - `/joint_commands` (`sensor_msgs/msg/JointState`): Target angles and PD gains.
  - `/mit_controller/commands` (`std_msgs/msg/Float64MultiArray`): Direct MIT impedance values.
  - `/jaguar/emergency_stop` (`std_msgs/msg/Bool`): Motor disable trigger.

## Execution

```bash
ros2 run jaguar_control robstride_can_node
```
