# PACE data collection for NXP Jaguar

This procedure prepares real Jaguar measurements for offline PACE system
identification. It follows PACE's data-collection/fitting workflow, but does
not assume the repository's ANYmal-D model or excitation settings transfer to
Jaguar unchanged.

## What this repo records

Run `scripts/pace/record_jaguar_joint_data.sh [label]` on a machine that has
the Jaguar ROS 2 topics available. The script only subscribes; it does not
publish commands or change motor state. It records:

- `/joint_states`: timestamped joint encoder position, velocity, and motor
  effort/torque feedback.
- `/joint_commands`: commanded joint positions and the controller's gain
  fields. In this driver the `JointState.effort` array on this topic encodes
  gains as `[Kp for 12 joints, Kd for 12 joints]`; it is not a measured torque.
- `/jaguar/motor_diagnostics` and `/jaguar/hardware_status`: temperature and
  hardware-status context.

The rosbag is saved under `data/pace/` (ignored by git). Preserve the original
bag unchanged and copy it to the offline identification workstation after the
recording.

## Before recording on the robot

1. Use the normal, known-good hardware launch and verify that `/joint_states`
   and `/joint_commands` are publishing with all 12 named joints.
2. Secure the robot in a supported test setup, clear the workspace, keep the
   emergency stop accessible, and have an operator ready to stop the test.
3. Record the robot/controller revision, payload, battery state, motor gains,
   and test setup alongside the bag label.
4. Start passive recording with:

   ```bash
   cd ~/jaguar_sim2real
   bash scripts/pace/record_jaguar_joint_data.sh bench_test
   ```

5. Stop with Ctrl+C after the measurement and confirm rosbag reports that it
   closed successfully. Do not commit recorded robot data to git.

This recorder does **not** create the PACE chirp/excitation. The README's
`scripts/pace/data_collection.py` generates a simulated ANYmal-D trajectory;
it is not a safe or compatible hardware excitation tool. Do not publish a
second `/joint_commands` stream alongside the Jaguar controller. A Jaguar
excitation method must be designed and reviewed separately, with bounded joint
targets, a supported test fixture, stop conditions, and explicit operator
control before any motor excitation is attempted.

## Data mapping and offline PACE input

PACE's fitting script expects a PyTorch file containing `time`, `dof_pos`, and
`des_dof_pos` arrays in one explicit joint order. Convert a rosbag on a machine
with ROS 2 bag Python packages, NumPy, and PyTorch installed:

```bash
cd ~/jaguar_sim2real
python3 scripts/pace/convert_rosbag_to_pace.py \
  data/pace/bench_test_<timestamp> \
  --output data/pace/nxp_jaguar_chirp.pt
```

The offline converter matches messages by joint **name**, maps hardware names
(`*_collar_joint`, `*_hip_joint`) to the Isaac/PACE names, interpolates measured
positions, and zero-order holds commands on a 200 Hz grid. It rejects unknown
or duplicate joints and timestamps with excessive gaps. The output tensors are
ordered leg-by-leg (`Fr`, `Fl`, `Br`, `Bl`), with roll, hip, and knee per leg.
Copy the result to
`pace-sim2real/data/nxp_jaguar/chirp_data.pt` before fitting.

**Do not fit ordinary walking or standing bags using the current fixed-base
PACE task.** The measured trajectory must come from a supported test setup and
excitation that match the simulator assumptions. This converter only reshapes
passive logs; it does not generate excitation, validate motor safety, or make
locomotion data suitable for system identification. A Jaguar-specific active
excitation is intentionally not included until its safe limits and test
procedure are reviewed.

Keep the controller/gain path identical between real measurement and
simulator replay. PACE's provided ANYmal-D actuator configuration is not a
Jaguar QDD configuration; validate the actuator model and parameter bounds
before fitting.

The default Jaguar ROS hardware loop publishes encoder feedback at 200 Hz;
the locomotion controller's joint targets are typically published at 50 Hz.
Inspect recorded rates and timestamp jitter before choosing a replay timestep.

## Current limits

- This repository contains a passive bag recorder and offline converter; no
  real-hardware excitation publisher is included.
- The PACE repository's reference task, actuator, joint names, and fitting
  bounds are for ANYmal D. They must be ported and checked for Jaguar before
  running `scripts/pace/fit.py`.
- Isaac Sim 6.1 is installed locally, while PACE's current classifiers list
  Isaac Sim 4.5/5.0. Import and a small Jaguar environment smoke test are still
  required before relying on PACE with the local simulator.
