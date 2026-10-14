# CyberGear protocol in the standalone Jaguar ROS node

## Source and integration

Six packet headers in `include/cybergear_driver_core/` and
`config/cybergear_motor_params.yaml` were copied from
`Pion-Robots/pion_ros2_control_plugin` commit
`e92866535d40b1549b8257fd808de9637c2fa9ce`, under `cybergear_ros2_control/`.
The headers retain their MIT license notices. The integration adapts those
classes in `include/jaguar_control/robstride_protocol.hpp`; it does not require
building `cybergear_control` or running a ROS controller manager.

The existing `robstride_can_node` executable is the standalone node. Its
SocketCAN transport, 12-motor bus layout, calibrated offsets, joint directions,
command validation, watchdog, and shutdown logic remain the Jaguar implementation.
Move commands, feedback conversion, frame IDs, run-mode writes, and torque-limit
writes use the copied CyberGear core. Stop frames are zero-initialized and retain
Jaguar's explicit optional fault-clear byte.

At startup the node loads the RS00 entry from the installed YAML using yaml-cpp,
before opening CAN sockets. The `motor_params_file` ROS parameter can select an
explicit configuration file. The YAML includes other motor types, but this node
supports RS00 only because its calibration and safety contract are for RS00.

| Quantity | Value |
| --- | --- |
| Position encoding | -12.566370614 to +12.566370614 rad |
| Velocity encoding | -33 to +33 rad/s |
| Torque encoding and decoding | -14 to +14 Nm |
| Kp encoding | 0 to 500 |
| Kd encoding | 0 to 5 |
| Temperature scale | 0.1 degrees C per count |
| YAML firmware torque ceiling | 14 Nm |
| Default Jaguar control and startup firmware cap | 13.5 Nm |
| Controller feedback overtorque threshold | 13.8 Nm |

The Python RS00 fallback uses the same torque range. The controller threshold
was lowered from 15 to 13.8 Nm because 15 Nm is outside the new feedback encoding
range; the operating cap remains 13.5 Nm. An old robot-side parameter file that
sets the threshold above 14 Nm must be updated and is rejected by the controller.

## Run

Build from a workspace that contains this package in `src/`:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select jaguar_control --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
ros2 launch jaguar_control robstride_standalone.launch.py
```

Alternatively run `ros2 run jaguar_control robstride_can_node` directly.
The node opens `can0` and `can1` and configures/enables motors on startup.
The driver-only launch does not launch an IMU, teleop, or policy controller.
The existing `sim2real.launch.py` starts the complete stack using the same node.

Commands enter through `/joint_commands` (`sensor_msgs/JointState`) or the
existing `/mit_controller/commands` array interface. Joint feedback is published
on `/joint_states`; the existing Jaguar diagnostic, stop, and reset interfaces
are retained. Rebuild and source the new installation before using it; editing
source alone does not update an existing installed executable.

## IMU source comparison

The comparison uses `serial_imu/src/serial_port.cpp` and `ch_serial.c` from
this repository and `sealand_ros2_control/src/imu_hardware_interface.cpp` and
`ch_serial.cpp` from the same pinned Pion submodule.

| Aspect | Jaguar serial_imu | Pion sealand_control |
| --- | --- | --- |
| Integration | Standalone publisher node | ros2_control SystemInterface plugin and IMU broadcaster |
| Serial protocol | CHAOHE sync 0x5A 0xA5, length and CRC16 | Same sync, length and CRC16 |
| Baud configuration | Fixed 115200 | Parameter; supplied URDF uses 921600 |
| Quaternion interpretation | Input w,x,y,z mapped to ROS fields | Same ordering in state interfaces |
| Angular velocity | degrees/s multiplied by 0.017453292519943295 | degrees/s multiplied by 0.01745329 |
| Acceleration | g multiplied by 9.80665 | g multiplied by 9.8 |
| Read scheduling | 2 ms timer, nonblocking serial reads | Controller manager read cycle; supplied config is 500 Hz |
| Output | sensor_msgs/Imu; default /Imu_data and imu_link | imu_sensor state interfaces; broadcaster creates the ROS message |
| Valid packet requirement | rev == 1, nonempty packet and IMU list | Uses truthiness of rev and unguarded last-item/last-IMU indexing |
| Measurement checks | Requires acceleration, gyro and quaternion; finite values and quaternion norm check | No equivalent checks in read() |
| Parser bounds | Validates item sizes/counts, resets decoded IMU fields per frame | No equivalent payload prevalidation/reset in parse_data() |
| Disconnect behavior | Closes on read errors and retries every second | Activation failure returns ERROR; no automatic reconnect in read() |

A 2 ms timer or a 500 Hz controller does not establish the physical IMU sample
rate. Neither wrapper applies a sensor-to-body mounting transform here; that
must be checked in the consumer and robot description.

In Sealand, `if (rev)` also accepts parser error -1. In addition,
`item_code[nitem_code - 1]` is accessed before checking for a complete packet,
so an initially empty item list can be indexed out of bounds. These are source
findings, not claims about a running sensor. Keep the existing standalone
serial_imu for this deployment; replacing it with Sealand would require matching
the configured baud and port and addressing the parser/read guards first.

## Validation status

The native node is compiled in an isolated build directory with ROS 2 Jazzy and
system Python. No motor node was launched and no CAN commands were sent.
No physical motor or IMU behavior, convergence, or locomotion quality is certified
by this source change. Automated protocol and motion-safety tests were not run.
