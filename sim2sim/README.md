# NXP Jaguar: Sim-to-Sim Validation (MuJoCo & Isaac Lab)

Verification pipeline for reinforcement learning locomotion policies (DreamWaQ) across MuJoCo and Isaac Lab physics engines prior to physical robot deployment.

## System Architecture

```
                          ┌──────────────────────────────┐
                          │   Trained JIT Actor Policy   │
                          │        (policy.pt)           │
                          └──────────────┬───────────────┘
                                         │ Action (12D)
                                         ▼
┌─────────────────────────┐   ┌──────────────────────────┐   ┌──────────────────────────┐
│   Teleop / Gamepad      │   │  50 Hz Control Loop      │   │     MuJoCo Physics       │
│  • Keyboard (Terminal)  ├──►│  • ObservationBuilder    ├──►│  • 500 Hz Decimation     │
│  • USB/BT Joystick      │   │  • Quintic Trajectory    │   │  • CAD Mesh Contact      │
└─────────────────────────┘   └──────────▲───────────────┘   │  • Camera Tracking       │
                                         │ Sensors           └──────────────────────────┘
                                         └───────────────────────────────┘
```

## Capabilities

1. **Authentic CAD Kinematic Model**:
   - Surface STL meshes: `Base_body.STL`, `*_coxa_roll.STL`, `*_hip_pitch.STL`, `*_tibia_pitch.STL`.
   - Measured mass ($5.2\text{ kg}$ base), link dimensions, and RobStride RS00 saturation limits ($17\text{ N m}$).
   - Direct mesh geometry contact modeling at foot pads.
2. **TorchScript Policy Loading**:
   - Loads exported TorchScript JIT policies (`policy.pt`) from Isaac Lab training checkpoints (`~/IsaacLab/logs/rsl_rl/nxp_jaguar_rough/`).
3. **Quintic Standup Trajectory**:
   - Smooth transition from zero angles ($q = 0.0\text{ rad}$) to nominal stance $q_0 = \pm 1.50\text{ rad}$ using a quintic polynomial profile $S(\alpha) = 10\alpha^3 - 15\alpha^4 + 6\alpha^5$ over 2.0 s.
4. **Isometric Camera Tracking**:
   - Viewer camera tracks robot base coordinates in an isometric third-person frame.
5. **Non-Blocking Teleoperation**:
   - Captures terminal keystrokes asynchronously without requiring Enter key confirmations.

## Observation Vector Specification (48-D)

The observation vector matches the Isaac Lab training configuration (`LocomotionVelocityRoughEnvCfg.observations.policy`):

| Index | Observation Component | Dimension | Mathematical Definition | Description |
|---|---|---|---|---|
| 1 | `base_lin_vel` | 3D | $R^T \mathbf{v}_{\text{world}}$ | Body frame linear velocity ($v_x, v_y, v_z$) |
| 2 | `base_ang_vel` | 3D | $\boldsymbol{\omega}_{\text{body}}$ | Body frame angular velocity ($\omega_x, \omega_y, \omega_z$) |
| 3 | `projected_gravity` | 3D | $R^T [0, 0, -1]^T$ | Projected gravitational acceleration in body frame |
| 4 | `velocity_commands` | 3D | $[v_x^{\text{cmd}}, v_y^{\text{cmd}}, \omega_z^{\text{cmd}}]$ | Commanded planar velocity from user |
| 5 | `joint_pos_rel` | 12D | $q_{\text{isaac}} - q_0$ | Joint angular position relative to nominal stance $q_0$ |
| 6 | `joint_vel_rel` | 12D | $\dot{q}_{\text{isaac}}$ | Joint angular velocity |
| 7 | `actions` | 12D | $a_{t-1}$ | Previous policy action output |
| **Total** | **Observation** | **48D** | float32 Tensor | Policy actor input |

## Joint Order Reconciliation

MuJoCo MJCF and Isaac Lab define differing joint orderings. The [`observation_builder.py`](./observation_builder.py) module handles bidirectional permutation:

- **Isaac Lab Sequence (Rolls -> Hips -> Knees)**:
  `[0: Fr_r, 1: Fl_r, 2: Br_r, 3: Bl_r, 4: Fr_h, 5: Fl_h, 6: Br_h, 7: Bl_h, 8: Fr_k, 9: Fl_k, 10: Br_k, 11: Bl_k]`
- **MuJoCo Sequence (Per Leg: FR -> FL -> BR -> BL)**:
  `[0..2: Fr_r, Fr_h, Fr_k | 3..5: Fl_r, Fl_h, Fl_k | 6..8: Br_r, Br_h, Br_k | 9..11: Bl_r, Bl_h, Bl_k]`
- **Permutation Vectors**:
  - `MUJOCO_TO_ISAAC = [0, 3, 6, 9, 1, 4, 7, 10, 2, 5, 8, 11]`
  - `ISAAC_TO_MUJOCO = [0, 4, 8, 1, 5, 9, 2, 6, 10, 3, 7, 11]`

## Actuator Parameters and Control Gains (RobStride RS00)

- **Nominal Stance Angles ($q_0$)**:
  - Roll: `0.00 rad`
  - Hip Pitch: `-1.50 rad`
  - Knee: `+1.50 rad`
- **Action Scaling**: 0.25 ($\text{Target } q = q_0 + 0.25 \times a_t$)
- **PD Impedance Gains**:
  - Locomotion Mode (`WALK`): $K_p = 25.0\text{ N m/rad}, K_d = 1.0\text{ N m s/rad}$
  - Stance Transition (`STANDUP`): $K_p = 35.0\text{ N m/rad}, K_d = 2.0\text{ N m s/rad}$
  - Torque Saturation Limit: $\tau_{\max} = 17.0\text{ N m}$
- **Loop Rates**:
  - Control Evaluation: 50 Hz ($\Delta t = 0.02\text{ s}$)
  - Physics Integration: 500 Hz ($\Delta t = 0.002\text{ s}$, Decimation = 10)

## Execution Guide

### 1. MuJoCo Simulation

Run simulation on selectable terrain models:

```bash
conda activate isaaclab
cd /home/erc/sim2real/sim2sim

# Flat Ground Plane
python3 sim2sim_mujoco.py --terrain flat

# Rough Terrain
python3 sim2sim_mujoco.py --terrain rough

# Stepped Stairs
python3 sim2sim_mujoco.py --terrain stairs

# Stepping Stones
python3 sim2sim_mujoco.py --terrain obstacles
```

Loading specific checkpoints:
```bash
python3 sim2sim_mujoco.py --terrain rough --load_run 2026-08-17_14-07-27
python3 sim2sim_mujoco.py --terrain rough --policy /path/to/exported/policy.pt
```

### 2. Isaac Lab Interactive Simulation

Interactive simulation with Viser visualization:

```bash
conda activate isaaclab
cd /home/erc/sim2real/sim2sim
python3 sim2sim_isaaclab.py --task flat --viz viser
```

## Teleoperation Guide

### Keyboard Controls

| Key | Mode / Command | Description |
| :---: | :--- | :--- |
| `1` | `STANDBY` | Sets all joint setpoints to zero ($q = 0.0\text{ rad}$) |
| `2` | `STANDUP` | Executes quintic trajectory to nominal stance $q_0 = \pm 1.5\text{ rad}$ |
| `3` | `WALK` | Activates 50 Hz DreamWaQ policy inference |
| `W / S` | Surge ($\pm v_x$) | Adjusts linear velocity in $\pm 0.2\text{ m/s}$ increments |
| `A / D` | Sway ($\pm v_y$) | Adjusts lateral velocity in $\pm 0.15\text{ m/s}$ increments |
| `Q / E` | Yaw ($\pm \omega_z$) | Adjusts yaw rate in $\pm 0.3\text{ rad/s}$ increments |
| `Space` | Brake | Clears velocity commands to $0.0$ |
| `R` | Reset | Resets simulation state to initial pose |

### Gamepad Controls

- Button A / Cross: Stance transition (`STANDUP`)
- Button B / Circle: Locomotion mode (`WALK`)
- Button X / Square: Passive rest (`STANDBY`)
- Left Analog Stick: Planar linear velocity ($v_x, v_y$)
- Right Analog Stick: Yaw rate ($\omega_z$)

## Directory Structure

```text
sim2sim/
├── README.md                 # Technical documentation
├── sim2sim_mujoco.py         # MuJoCo simulation environment
├── sim2sim_isaaclab.py       # Isaac Lab simulation environment
├── observation_builder.py    # 48-D observation tensor construction and kinematics
└── models/
    ├── scene.xml             # MuJoCo lighting, floor, and visual scene
    ├── nxp_jaguar.xml        # MJCF robot description
    ├── urdf/
    │   └── nxp_jaguar.urdf   # URDF kinematic tree
    └── meshes/               # CAD STL surface meshes
```

## Sim-to-Real Equivalence

The sim-to-sim pipeline mirrors onboard physical robot control interfaces:
1. State Estimation: IMU orientation quaternion -> body frame velocities -> projected gravity vector.
2. Actuator Commands: Scaled policy output $[-1, 1] \times 0.25 + q_0 \to$ RobStride RS00 SocketCAN.
3. Finite State Machine: Sequential transitions `STANDBY` -> `STANDUP` -> `WALK`.
