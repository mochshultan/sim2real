# RobStride C++ Driver Headers (`include/jaguar_control/`)

C++ headers for SocketCAN communication and RobStride RS00 motor control on the NXP Jaguar quadruped.

## Headers

| Header File | Responsibilities | Key Symbols |
| :--- | :--- | :--- |
| [`robstride_protocol.hpp`](./robstride_protocol.hpp) | Bit-packing, linear scaling, 29-bit CAN arbitration ID assembly, and telemetry frame parsing. | `struct MotorParams`, `struct MotorFeedback`, `buildMitControlFrame()`, `buildEnableMotorFrame()`, `buildStopMotorFrame()`, `buildSetRunModeFrame()`, `parseFeedbackFrame()` |
| [`robstride_can_bus.hpp`](./robstride_can_bus.hpp) | Linux SocketCAN wrapper with non-blocking I/O. | `class RobStrideCanBus` (`openBus()`, `closeBus()`, `sendFrame()`, `receiveFrame()`) |
| [`robstride_hardware_manager.hpp`](./robstride_hardware_manager.hpp) | Dual CAN bus coordination (`can0` and `can1`), joint command dispatch, watchdog timeout monitoring, and zero-torque fallback. | `class RobStrideHardwareManager`, `struct JointConfig`, `struct JointCommand`, `struct JointStateData` |

## Design Constraints

- **Fixed Allocation**: Packs bits into `struct can_frame` without heap allocations in the control loop.
- **Non-Blocking I/O**: Sockets set `O_NONBLOCK` for the 200 Hz CAN cycle.
- **Fail-Safe**: Switches to zero torque when command delay exceeds 100 ms or on emergency stop.
