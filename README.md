# NXP Jaguar Quadruped: Sim-to-Real Deployment (Branch: `cpp`)

Reinforcement learning control deployment from Isaac Lab (DreamWaQ) to RobStride RS00 actuators via ROS 2.

## Overview

The `cpp` branch executes low-level actuator communication through a native Linux SocketCAN driver node (`robstride_can_node`) running under a real-time `SCHED_FIFO` priority 80 thread. The node operates deterministically at 200 Hz, coordinating 12 RobStride RS00 Quasi-Direct Drive (QDD) actuators with sub-millisecond transmission latency.

## Table of Contents
1. [System Architecture](#1-system-architecture)
2. [C++ Driver Modules](#2-c-driver-modules)
3. [RL Policy Observation Space (48-D)](#3-rl-policy-observation-space-48-d)
4. [Joint Index Remapping (Isaac Lab vs. Hardware)](#4-joint-index-remapping-isaac-lab-vs-hardware)
5. [Actuator CAN Bus and Node ID Mapping](#5-actuator-can-bus-and-node-id-mapping)
6. [Actuator Parameters and PD Impedance Gains](#6-actuator-parameters-and-pd-impedance-gains)
7. [Teleoperation Interface](#7-teleoperation-interface)
8. [Hardware Bringup Procedure](#8-hardware-bringup-procedure)
9. [Diagnostic and Calibration Utilities](#9-diagnostic-and-calibration-utilities)
10. [Safety Systems and Failsafes](#10-safety-systems-and-failsafes)
11. [MuJoCo Sim-to-Sim Validation](#11-mujoco-sim-to-sim-validation)
12. [Kinematic and Reward Formulation Notes](#12-kinematic-and-reward-formulation-notes)

## 1. System Architecture

```
[ HIGH-LEVEL: RL Policy ]
      │  Model: TorchScript JIT (`policy.pt`) trained in Isaac Lab 3.0 (DreamWaQ)
      │  Rate: 50 Hz (dt = 0.02 s) | Input: 48-D Observation | Output: 12-D Target Δq
      ▼
[ MID-LEVEL: ROS 2 Controller Node (`scripts/nxp_jaguar_controller.py`) ]
      │  • Subscriptions: IMU (`/Imu_data`), Joy/Teleop (`/joy`, `/cmd_vel`), Joint States (`/robot_joint_states`)
      │  • Finite State Machine: STANDBY ──(Btn A)──> STANDUP ──(Btn B)──> WALK ──(Btn X)──> E-STOP
      │  • Remapping: Isaac Order (Roll->Hip->Knee) ⇄ ROS Hardware Order (BL->BR->FL->FR)
      │  • Target Position: q_des = q_nominal + 0.25 * action
      │  • Publication: `/joint_command` (std_msgs/Float64MultiArray)
      ▼
[ LOW-LEVEL: Real-Time C++ CAN Node (`src/robstride_can_node.cpp`) ]
      │  • Dual SocketCAN Threads (`can0` and `can1`) via `robstride_can_bus.hpp`
      │  • Real-Time Scheduler: Linux `SCHED_FIFO` (Priority 80)
      │  • Protocol: RobStride RS00 bit-packed frames (`robstride_protocol.hpp`)
      │  • Deterministic Loop Rate: 200 Hz (dt = 0.005 s)
      │  • Feedback Publication: `/robot_joint_states` (sensor_msgs/JointState)
      ▼
[ HARDWARE: 12x RobStride RS00 Actuators and Hiwonder 9-DOF IMU ]
```

## 2. C++ Driver Modules

C++ source headers reside in [`include/jaguar_control/`](file:///home/erc/sim2real/include/jaguar_control/) and executable implementations in [`src/`](file:///home/erc/sim2real/src/):

| File | Primary Interface | Description |
| :--- | :--- | :--- |
| **`robstride_protocol.hpp`** | `RobStrideProtocol` | Encodes floating-point targets ($q, \dot{q}, K_p, K_d, \tau_{\text{ff}}$) into 16-bit packed payloads and parses 29-bit CAN arbitration responses. |
| **`robstride_can_bus.hpp`** | `RobStrideCANBus` | Manages non-blocking POSIX SocketCAN raw sockets (`AF_CAN`, `SOCK_RAW`) with hardware frame filters. |
| **`robstride_hardware_manager.hpp`** | `RobStrideHardwareManager` | Coordinates dual CAN channels (`can0` right 6 motors, `can1` left 6 motors), generates quintic startup trajectories, interpolates setpoints, and compensates encoder zero offsets. |
| **`robstride_can_node.cpp`** | `RobStrideCANNode` | ROS 2 wrapper thread running under `SCHED_FIFO` at 200 Hz, bridging ROS topics to the low-level hardware manager. |

## 3. RL Policy Observation Space (48-D)

The actor policy ingests a 48-dimensional observation vector at 50 Hz:

| Index Range | State Variable | Dimension | Unit | Description |
| :---: | :--- | :---: | :---: | :--- |
| `[0 : 3]` | `base_lin_vel` | 3 | $\text{m/s}$ | Base linear velocity in body frame $[v_x, v_y, v_z]$ |
| `[3 : 6]` | `base_ang_vel` | 3 | $\text{rad/s}$ | Base angular velocity in body frame $[\omega_x, \omega_y, \omega_z]$ |
| `[6 : 9]` | `projected_gravity` | 3 | unit | Projected gravity vector $[g_x, g_y, g_z]$ ($[0, 0, -1]$ upright) |
| `[9 : 12]` | `velocity_commands` | 3 | $\text{m/s, rad/s}$ | Commanded planar velocity $[v_x^{\text{cmd}}, v_y^{\text{cmd}}, \omega_z^{\text{cmd}}]$ |
| `[12 : 24]` | `joint_pos_rel` | 12 | $\text{rad}$ | Joint position relative to nominal: $(q_i - q_{0, i})$ |
| `[24 : 36]` | `joint_vel` | 12 | $\text{rad/s}$ | Joint angular velocity $\dot{q}_i$ |
| `[36 : 48]` | `actions` | 12 | $\text{rad}$ | Previous policy output $a_{t-1}$ |

$$\text{Total Observation Dimension} = 3 + 3 + 3 + 3 + 12 + 12 + 12 = 48$$

## 4. Joint Index Remapping (Isaac Lab vs. Hardware)

- **Hardware CAN Driver**: Grouped leg-by-leg (`BL`, `BR`, `FL`, `FR`).
- **Isaac Lab Policy**: Grouped joint-by-joint (Rolls, Hips, Knees).

```
[Hardware CAN Driver Order (C++ / ROS)]      [Isaac Lab Policy Order]
Grouped by LEG (BL, BR, FL, FR)              Grouped by JOINT TYPE (Rolls, Hips, Knees)
───────────────────────────────────────      ──────────────────────────────────────────
 0: BL_collar_joint                           0: Fr_roll_joint   (Front Right Roll)
 1: BL_hip_joint                              1: Fl_roll_joint   (Front Left Roll)
 2: BL_knee_joint                             2: Br_roll_joint   (Back Right Roll)
 3: BR_collar_joint                           3: Bl_roll_joint   (Back Left Roll)
 4: BR_hip_joint                              4: Fr_hip_pitch    (Front Right Hip)
 5: BR_knee_joint                             5: Fl_hip_pitch    (Front Left Hip)
 6: FL_collar_joint                           6: Br_hip_pitch    (Back Right Hip)
 7: FL_hip_joint                              7: Bl_hip_pitch    (Back Left Hip)
 8: FL_knee_joint                             8: Fr_knee_joint   (Front Right Knee)
 9: FR_collar_joint                           9: Fl_knee_joint   (Front Left Knee)
10: FR_hip_joint                             10: Br_knee_joint   (Back Right Knee)
11: FR_knee_joint                            11: Bl_knee_joint   (Back Left Knee)
```

Permutation arrays in `scripts/nxp_jaguar_controller.py`:
```python
ROS_TO_ISAAC = [9, 6, 3, 0, 10, 7, 4, 1, 11, 8, 5, 2]
ISAAC_TO_ROS = [3, 7, 11, 2, 6, 10, 1, 5, 9, 0, 4, 8]
```

## 5. Actuator CAN Bus and Node ID Mapping

| Isaac Index | Isaac Joint Name | Nominal Angle ($q_0$) | ROS Index | ROS Joint Name (`parameters.py`) | CAN ID | Bus | Quadrant |
| :---: | :--- | :---: | :---: | :--- | :---: | :---: | :---: |
| **0** | `Fr_roll_joint` | $0.0\text{ rad}$ | **9** | `FR_collar_joint` | **1** | `can0` | Front Right |
| **1** | `Fl_roll_joint` | $0.0\text{ rad}$ | **6** | `FL_collar_joint` | **1** | `can1` | Front Left |
| **2** | `Br_roll_joint` | $0.0\text{ rad}$ | **3** | `BR_collar_joint` | **4** | `can0` | Rear Right |
| **3** | `Bl_roll_joint` | $0.0\text{ rad}$ | **0** | `BL_collar_joint` | **4** | `can1` | Rear Left |
| **4** | `Fr_hip_pitch_joint` | $-1.50\text{ rad}$ | **10** | `FR_hip_joint` | **2** | `can0` | Front Right |
| **5** | `Fl_hip_pitch_joint` | $-1.50\text{ rad}$ | **7** | `FL_hip_joint` | **2** | `can1` | Front Left |
| **6** | `Br_hip_pitch_joint` | $-1.40\text{ rad}$ | **4** | `BR_hip_joint` | **5** | `can0` | Rear Right |
| **7** | `Bl_hip_pitch_joint` | $-1.40\text{ rad}$ | **1** | `BL_hip_joint` | **5** | `can1` | Rear Left |
| **8** | `Fr_knee_joint` | $+1.40\text{ rad}$ | **11** | `FR_knee_joint` | **3** | `can0` | Front Right |
| **9** | `Fl_knee_joint` | $+1.40\text{ rad}$ | **8** | `FL_knee_joint` | **3** | `can1` | Front Left |
| **10** | `Br_knee_joint` | $+1.36\text{ rad}$ | **5** | `BR_knee_joint` | **6** | `can0` | Rear Right |
| **11** | `Bl_knee_joint` | $+1.36\text{ rad}$ | **2** | `BL_knee_joint` | **6** | `can1` | Rear Left |

## 6. Actuator Parameters and PD Impedance Gains

- **Policy Evaluation Rate**: 50 Hz ($\Delta t = 0.02\text{ s}$)
- **Low-Level Hardware Rate**: 200 Hz ($\Delta t = 0.005\text{ s}$)
- **Real-Time Scheduling**: Linux `SCHED_FIFO`, Priority 80
- **Action Scaling**: 0.25 ($q_{\text{des}} = q_0 + 0.25 \times a_{\text{policy}}$)
- **Joint Stiffness ($K_p$)**: 25.0 N m/rad
- **Joint Damping ($K_d$)**: 1.5 N m s/rad
- **Actuator Torque Limit**: 17.0 N m

## 7. Teleoperation Interface

### A. Keyboard Interface (`scripts/keyboard_teleop.py`)

Run the keyboard teleoperation node:
```bash
ros2 run jaguar_control keyboard_teleop.py
```

| Key | Mode / Command | Action |
| :---: | :--- | :--- |
| `1` | `STANDBY` | Drives all joints to zero position ($0.0\text{ rad}$) and clears velocity targets. |
| `2` | `STANDUP` | Executes a 2-second quintic S-curve trajectory to nominal stance angles $q_0$. |
| `3` | `WALK` | Activates 50 Hz neural network policy inference. |
| `W` | Surge Forward ($+v_x$) | Increments forward velocity by $+0.1\text{ m/s}$ (max $+1.2\text{ m/s}$). |
| `S` | Surge Backward ($-v_x$) | Increments backward velocity by $-0.1\text{ m/s}$ (max $-0.8\text{ m/s}$). |
| `A` | Sway Left ($+v_y$) | Increments lateral velocity by $+0.1\text{ m/s}$ (max $+0.5\text{ m/s}$). |
| `D` | Sway Right ($-v_y$) | Increments lateral velocity by $-0.1\text{ m/s}$ (max $-0.5\text{ m/s}$). |
| `Q` | Yaw Left ($+\omega_z$) | Increments counterclockwise yaw by $+0.2\text{ rad/s}$ (max $+1.2\text{ rad/s}$). |
| `E` | Yaw Right ($-\omega_z$) | Increments clockwise yaw by $-0.2\text{ rad/s}$ (max $-1.2\text{ rad/s}$). |
| `X` | Brake | Sets planar command velocities to zero while keeping policy active. |
| `SPACE` | Emergency Stop | Cuts motor commands and commands transition to `STANDBY`. |

### B. Gamepad Interface (`/dev/input/js0`)

- `A` / `X` (Cross): Stance transition (`STANDUP`)
- `B` / `Circle`: Locomotion mode (`WALK`)
- `X` / `Square`: Passive rest (`STANDBY`)
- Left Analog Stick (Vertical / Horizontal): Planar linear velocity ($v_x, v_y$)
- Right Analog Stick (Horizontal): Yaw rate ($\omega_z$)

## 8. Hardware Bringup Procedure

> [!IMPORTANT]
> 1. Suspend the chassis on a gantry until all four feet clear the floor before powering actuators.
> 2. Connect the 24V supply and verify the hardware emergency stop switch.

### Initial Build
```bash
cd /home/erc/sim2real
colcon build --packages-select jaguar_control --symlink-install
source install/setup.bash
```

### Multi-Terminal Bringup

Launch nodes in separate terminal sessions in order:

#### Terminal 1: CAN Network Setup (1 Mbps)
```bash
cd /home/erc/sim2real
sudo ./scripts/bringup_canbus.sh
```

#### Terminal 2: Serial IMU Node
```bash
cd /home/erc/sim2real
./scripts/bringup_imu.sh
```

#### Terminal 3: Real-Time C++ CAN Node (200 Hz)
```bash
cd /home/erc/sim2real
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run jaguar_control robstride_can_node
```

#### Terminal 4: Sensor Diagnostics Dashboard
```bash
cd /home/erc/sim2real
source /opt/ros/humble/setup.bash
python3 scripts/check_states.py
```

#### Terminal 5: Sim-to-Real RL Controller Node
```bash
cd /home/erc/sim2real
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run jaguar_control nxp_jaguar_controller.py
```

#### Terminal 6: Teleoperation Hub
```bash
cd /home/erc/sim2real
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run jaguar_control keyboard_teleop.py
```

Execution sequence:
1. Press **2**: Robot transitions to nominal stance (`STANDUP`).
2. Press **3**: Robot enables RL locomotion policy (`WALK`).
3. Steer with **W / S / A / D / Q / E**.
4. Press **1** to sit down (`STANDBY`), or **SPACE** for Emergency Stop.

### Monolithic Bringup (ROS 2 Launch)

```bash
# 1. Bring up CAN interfaces:
sudo /home/erc/sim2real/scripts/bringup_canbus.sh

# 2. Launch full stack:
source /opt/ros/humble/setup.bash
source /home/erc/sim2real/install/setup.bash
ros2 launch jaguar_control sim2real.launch.py
```

## 9. Diagnostic and Calibration Utilities

> [!NOTE]
> See [ZERO_CALIBRATION_GUIDE.md](file:///home/erc/sim2real/ZERO_CALIBRATION_GUIDE.md) for encoder offset calibration on the 12 actuators.

- **CAN ID Discovery**:
  ```bash
  python3 scripts/scan_robostride_ids.py
  ```
- **Mechanical Zero Calibration**:
  Place legs in the Relax Pose, then run:
  ```bash
  python3 scripts/set_robostride_zero.py
  ```
- **Sit-Stand Trajectory Validation**:
  ```bash
  python3 scripts/test_sit_stand.py
  ```
- **Passive Joint Encoder Readout**:
  ```bash
  python3 scripts/check_joints.py
  ```

## 10. Safety Systems and Failsafes

1. **Orientation Watchdog (Tilt Protection)**: If projected gravity $g_z > -0.5$ (tilt angle $> 60^\circ$), the controller disables locomotion and returns to `STANDBY`.
2. **Communication Watchdog**: If IMU or CAN communication delays exceed 0.1 s, the node shuts off motor torques.
3. **Thermal Protection**: Actuator diagnostic routines issue warnings when motor temperature exceeds $75^\circ\text{C}$.

## 11. MuJoCo Sim-to-Sim Validation

Validate the RL locomotion policy in MuJoCo:
```bash
python3 sim2sim/sim2sim_mujoco.py --terrain flat
python3 sim2sim/sim2sim_mujoco.py --terrain rough
```

Keyboard mapping in MuJoCo:
- `1`: Standby (zero angles)
- `2`: Stand Up (quintic trajectory)
- `3`: Walk (RL policy active)
- `W / S / A / D / Q / E`: Planar velocity commands

## 12. Kinematic and Reward Formulation Notes

### 1. True Foot Tip Clearance Kinematics
The URDF `tibia_link` origin frame resides at the proximal knee joint ($Z \approx 0.44\text{ m}$). Measuring clearance from the link origin fails to reflect physical ground clearance at the foot pad.

The distal contact point is computed from the lowest mesh vertex in `Fr_tibia_pitch.STL`:
$$\mathbf{p}_{\text{tip\_local}} = [+\mathbf{0.087\text{ m}}, \mathbf{0.0\text{ m}}, -\mathbf{0.1634\text{ m}}]$$

The reward function `foot_clearance_dreamwaq` maps this offset to the world frame using the tibia link rotation matrix $\mathbf{R}_{\text{tibia}}(q)$:
$$\mathbf{p}_{\text{foot\_tip, world}} = \mathbf{p}_{\text{tibia, world}} + \mathbf{R}_{\text{tibia}}(\mathbf{p}_{\text{tip\_local}})$$

This formulation enforces physical foot clearance during the swing phase.

### 2. Base Height Ground Clearance
When the chassis collapses to the floor ($\approx 10\text{ cm}$), the entire shank rests horizontally. Contact sensors trigger across the shank surface, creating a false stance height estimation ($\approx 23\text{ cm}$).

The reward `base_height_l2_safe` calculates true vertical clearance between the base origin and the ground plane ($Z_{\text{root}} - Z_{\text{ground}}$), anchoring stance height at **0.24 m**.

### 3. Stand-Still Joint Deviation Penalty
When planar velocity commands drop below threshold ($\|\mathbf{v}_{\text{cmd}}\| < 0.1\text{ m/s}$), the `stand_still` reward penalizes joint deviations from nominal stance $q_0$ (`Hips = -1.55 rad`, `Knees = 1.35 rad`, `Rolls = 0.0 rad`). This prevents standing drift and limit-cycle oscillations while stationary.
