# ⚡ RobStride C++ Real-Time Driver Headers (`include/jaguar_control/`)

High-performance, hard real-time C++ header library for SocketCAN communication and motor control of **RobStride RS00** actuators on the **NXP Jaguar Quadruped**.

---

## 📁 Headers Overview

| Header File | Primary Responsibilities | Key Classes / Functions |
| :--- | :--- | :--- |
| [`robstride_protocol.hpp`](./robstride_protocol.hpp) | CAN Protocol bit-packing, linear scaling, 29-bit CAN arbitration ID assembly, and telemetry frame parsing. | `struct MotorParams`, `struct MotorFeedback`, `buildMitControlFrame()`, `buildEnableMotorFrame()`, `buildStopMotorFrame()`, `buildSetRunModeFrame()`, `parseFeedbackFrame()` |
| [`robstride_can_bus.hpp`](./robstride_can_bus.hpp) | POSIX Linux SocketCAN RAII encapsulation with non-blocking I/O for deterministic sub-millisecond bus access. | `class RobStrideCanBus` (`openBus()`, `closeBus()`, `sendFrame()`, `receiveFrame()`) |
| [`robstride_hardware_manager.hpp`](./robstride_hardware_manager.hpp) | Multi-bus manager coordinating dual CAN buses (`can0` & `can1`), thread-safe joint command dispatch, watchdog timeout monitoring, and zero-torque passive fallback. | `class RobStrideHardwareManager`, `struct JointConfig`, `struct JointCommand`, `struct JointStateData` |

---

## 🏎️ Performance Characteristics

- **Zero-Copy Memory Layout**: Custom bit-packing into standard `struct can_frame` without heap allocations during active loop cycles.
- **Deterministic Latency**: Pure non-blocking socket operations (`O_NONBLOCK`) allowing steady **200 Hz** CAN cycle time.
- **Safety Defaults**: Automatic transition to passive zero-torque mode upon command timeouts (> 100 ms) or emergency stop signals.
