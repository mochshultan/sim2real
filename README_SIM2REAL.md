# NXP Jaguar Quadruped: Sim-to-Real Deployment Guide

Reinforcement learning control deployment from Isaac Lab (DreamWaQ) to RobStride RS00 hardware via ROS 2.

## Table of Contents
1. [Policy Observation Space (48-D)](#1-policy-observation-space-48-d)
2. [Coordinate Frame and Joint Order Remapping](#2-coordinate-frame-and-joint-order-remapping)
3. [Actuator Mapping and CAN Bus Configuration](#3-actuator-mapping-and-can-bus-configuration)
4. [Actuator Parameters and PD Gains](#4-actuator-parameters-and-pd-gains)
5. [Teleoperation Configuration](#5-teleoperation-configuration)
6. [Hardware Bringup Procedure](#6-hardware-bringup-procedure)
7. [Safety Systems and Failsafes](#7-safety-systems-and-failsafes)

## 1. Policy Observation Space (48-D)

The neural network actor policy ingests a 48-dimensional observation vector with the following layout:

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

## 2. Coordinate Frame and Joint Order Remapping

### Joint Grouping Divergence
The hardware CAN driver indexes actuators leg-by-leg (`BL`, `BR`, `FL`, `FR`), whereas the Isaac Lab policy indexes actuators joint-by-joint (Rolls, Hips, Knees).

```
[Hardware CAN Driver Order]                  [Isaac Lab Policy Order]
Grouped by LEG (BL, BR, FL, FR)              Grouped by JOINT TYPE (Rolls, Hips, Knees)
─────────────────────────────────────        ──────────────────────────────────────────
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

### Observation Normalizer Statistics (`policy.pt`)
The `obs_normalizer` buffer inside `policy.pt` reveals the training distribution:
- Indices `[6 : 9]` (Gravity): Mean $[0.0039, -0.0009, -0.9989]$. Upright orientation corresponds to $[0, 0, -1]$.
- Indices `[12 : 15]` (4 Roll joints): $\sigma \approx 0.13\text{ rad}$.
- Indices `[16 : 19]` (4 Hip joints): $\sigma \approx 0.16\text{ rad}$.
- Indices `[20 : 23]` (4 Knee joints): $\sigma \approx 0.12\text{ rad}$.

### Controller Remapping Pipeline
The controller manages joint order reconciliation through two stages:
1. Joint Name Lookup: Maps ROS topic joint names (`FR_collar_joint`) to Isaac tensor indices `[0..11]`.
2. Permutation Vectors: Reorders arrays if joint names are absent:
   ```python
   # Hardware CAN order (BL, BR, FL, FR) to Isaac policy order (Roll, Hip, Knee):
   ROS_TO_ISAAC = [9, 6, 3, 0, 10, 7, 4, 1, 11, 8, 5, 2]

   # Isaac policy output to hardware CAN order:
   ISAAC_TO_ROS = [3, 7, 11, 2, 6, 10, 1, 5, 9, 0, 4, 8]
   ```

## 3. Actuator Mapping and CAN Bus Configuration

| Isaac Index | Isaac Joint Name | Nominal Angle ($q_0$) | ROS Index | ROS Joint Name | CAN ID | Bus | Quadrant |
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
| **10** | `Br_knee_joint` | $+1.60\text{ rad}$ | **5** | `BR_knee_joint` | **6** | `can0` | Rear Right |
| **11** | `Bl_knee_joint` | $+1.60\text{ rad}$ | **2** | `BL_knee_joint` | **6** | `can1` | Rear Left |

## 4. Actuator Parameters and PD Gains

| Parameter | Value | Details |
| :--- | :---: | :--- |
| Policy Rate | 50 Hz | $\Delta t = 0.02\text{ s}$ |
| CAN Loop Rate | 200 Hz | Linux SocketCAN |
| Action Scale | 0.25 | $q_{\text{des}} = q_0 + 0.25 \times a_{\text{policy}}$ |
| Joint Stiffness ($K_p$) | 25.0 N m/rad | Position gain |
| Joint Damping ($K_d$) | 1.0 N m s/rad | Velocity damping gain |
| Actuator Torque Limit | 17.0 N m | Saturation limit |
| Knee Limits | `[-0.1, 2.8] rad` | Four-bar linkage mechanical bounds |
| Roll Limits | `[-0.4, 0.4] rad` | Collar sweep limits |

## 5. Teleoperation Configuration

### Supported Hardware
The controller reads standard `/dev/input/js0` devices via the ROS 2 `joy` package:
- Sony PlayStation 4 / DualShock 4
- Microsoft Xbox 360 / Xbox One / Xbox Series
- Logitech F710 / generic HID gamepads

### Button Mapping

| PS4 Button | Xbox Button | ROS Index (`msg.buttons`) | Action |
| :---: | :---: | :---: | :--- |
| `X` | `A` | `0` | `STANDUP`: Trajectory transition to stance angles $q_0$ over 2 s |
| `Circle` | `B` | `1` | `WALK`: Activates 50 Hz policy inference |
| `Square` | `X` | `2` | `STANDBY`: Clears velocity commands and commands sitting position |

### Analog Axes

| Analog Axis | Axis Index | Range | Commanded Motion |
| :--- | :---: | :---: | :--- |
| Left Stick (Vertical) | `axes[1]` | $[-1.0, 1.0]$ | Surge velocity ($v_x$) |
| Left Stick (Horizontal) | `axes[0]` | $[-0.5, 0.5]$ | Sway velocity ($v_y$) |
| Right Stick (Horizontal) | `axes[3]` or `axes[2]` | $[-1.2, 1.2]$ | Yaw rate ($\omega_z$) |

### Headless Teleoperation

#### Terminal Gamepad Emulator
```bash
python3 /home/erc/virtual_gamepad_headless.py
```
Key commands:
- `a`: Stand Up
- `b`: Walk (active policy)
- `x`: Standby (sit)
- `ly 0.5`: Forward surge $0.5\text{ m/s}$

#### Manual Topic Publication
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.4, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 20
```

## 6. Hardware Bringup Procedure

> [!IMPORTANT]
> 1. Suspend the chassis on a gantry until all four feet clear the floor before testing.
> 2. Connect the 24V supply and verify the emergency cutoff switch.

### Multi-Terminal Sequence

#### Terminal 1: CAN Bus Bringup (`can0` and `can1`)
```bash
cd /home/erc/sim2real
sudo ./scripts/bringup_canbus.sh
```

#### Terminal 2: Serial IMU Driver
```bash
cd /home/erc/sim2real
./scripts/bringup_imu.sh
```

#### Terminal 3: RobStride CAN Hardware Node
```bash
cd /home/erc/sim2real
source /opt/ros/humble/setup.bash
python3 scripts/can_hardware_node.py
```

#### Terminal 4: Sensor State Dashboard
```bash
cd /home/erc/sim2real
source /opt/ros/humble/setup.bash
python3 scripts/check_states.py
```
Manually rotate each leg to verify matching joint telemetry on the terminal display.

#### Terminal 5: RL Sim-to-Real Controller
```bash
cd /home/erc/sim2real
source /opt/ros/humble/setup.bash
python3 scripts/nxp_jaguar_controller.py
```

## 7. Safety Systems and Failsafes

1. **Orientation Watchdog**: If projected gravity $g_z > -0.5$ (tilt $> 60^\circ$), the controller disables locomotion and commands `STANDBY`.
2. **Communication Watchdog**: If IMU or CAN packet latency exceeds 0.1 s, the controller halts actuator output.
3. **Thermal Guard**: The system logs warnings when actuator thermistors report temperatures exceeding $75^\circ\text{C}$.
