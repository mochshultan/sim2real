# Jaguar PACE notes for the training and simulation PC

This note describes the handoff from the robot workstation to the PC used for
Isaac Lab simulation and PACE system identification. The real dataset below was
collected with the Jaguar **fully suspended in the air**. Keep that condition
attached to the data: the robot was not standing on the floor or walking.

## Retained robot run

The retained complete run is:

- Rosbag: `data/pace/jaguar_chirp_20261012_145729/`
- Command trajectory: `data/pace/jaguar_chirp_commands_20261012_145733.csv`
- Active joints: FR collar, hip, and knee
- Other nine joints: held at the nominal standing pose
- Gains: Kp=40, Kd=2.5
- Chirp amplitude: +/-0.03 rad
- Frequency sweep: 0.1 to 0.8 Hz over 20 seconds
- Command rate: 50 Hz

The robot was hoisted with all feet clear of the ground during both the
all-joint transition to the nominal pose and the FR chirp. Do not interpret
“nominal stand” as ground contact. Do not combine this recording with the
aborted 1 Hz run or with ordinary walking data.

Keep the raw rosbag and CSV unchanged. The bag includes the run context and
joint commands; the CSV contains the commanded transition and chirp phases.
When preparing the fit dataset, select only the FR chirp interval. Exclude the
stand transition, pre-chirp hold, and any post-stop/watchdog tail. Align the
bag and CSV by their timestamps/command trace; their time origins are not
guaranteed to match.

## Copy the data to the PC

Transfer both retained files to the PC, for example with `scp` over the
robot-workstation network:

```bash
scp -r erc@<robot-host>:/home/erc/sim2real/data/pace/jaguar_chirp_20261012_145729 \
  <pc-user>@<pc-host>:~/jaguar_pace_data/
scp erc@<robot-host>:/home/erc/sim2real/data/pace/jaguar_chirp_commands_20261012_145733.csv \
  <pc-user>@<pc-host>:~/jaguar_pace_data/
```

Replace the host and account placeholders with the actual machine names. Keep
the original bag as an archival copy. The robot data directory is ignored by
Git; these binary recordings are transferred separately rather than added to
the source commit.

## Convert the rosbag

The converter needs ROS 2 bag Python support, NumPy, and PyTorch. On the PC,
source its ROS 2 environment, enter a checkout of this repository, and run:

```bash
source /opt/ros/humble/setup.bash
cd ~/sim2real
python3 scripts/pace/convert_rosbag_to_pace.py \
  ~/jaguar_pace_data/jaguar_chirp_20261012_145729 \
  --output ~/jaguar_pace_data/nxp_jaguar_chirp_full.pt
```

The converter maps joints by name and writes PACE tensors in leg-by-leg order:
FR, FL, BR, BL; roll, hip, knee for each leg. It resamples to 200 Hz. Its
output includes the whole bag overlap, so trim it to the chirp interval before
fitting. Fit only the three excited FR joints, or mask the other nine joints
from the fitting loss. Unexcited joints cannot identify their actuator
parameters.

Copy the trimmed tensor to the PACE checkout at
`data/nxp_jaguar/chirp_data.pt`, matching the path configured for the custom
Jaguar task. Preserve its `time`, `dof_pos`, and `des_dof_pos` arrays and
joint order. Do not silently reorder columns or interpret commanded gains as
measured torque.

## Prepare matching simulation data

The upstream PACE example is for ANYmal D. Do not run its default
`scripts/pace/data_collection.py` unchanged for Jaguar: its task, joint
offsets, ordering, and excitation settings do not match this robot.

Before fitting, the PC checkout needs a Jaguar-specific Isaac Lab/PACE task and
a simulation replay that uses the same experiment:

1. Load the Jaguar model with the calibrated joint offsets and matching joint
   limits. Use the same nominal pose as the real run.
2. Replay the exact FR command trajectory from the retained CSV. Keep the other
   nine joints holding the nominal pose. Use the same joint order, radians,
   50 Hz command updates, 0.1 to 0.8 Hz chirp, and +/-0.03 rad excursion.
3. Use the same actuator command path and Kp=40, Kd=2.5 in simulation and on
   the robot. Match the initial joint positions to the measured bag values at
   the beginning of the chirp.
4. Record simulated measured joint position and desired joint position, then
   export them with the same `time`, `dof_pos`, and `des_dof_pos` schema and
   joint order as the real tensor.
5. Compare the three FR measured and desired trajectories before fitting.
   Check units, sign, offsets, chirp phase, sample rate, and time alignment.

The simulation replay should represent the same suspended test: do not add
ground contacts or use a walking policy for this fixed-pose identification
trial. The stand-transition CSV rows are not part of the chirp fit interval.

## PACE environment and fitting status

Install the Isaac Lab and Isaac Sim versions required by the PACE checkout on
the PC, then install PACE into that environment. Verify that the Jaguar task
imports and runs a short replay before launching an optimizer. The workspace
that collected this run contains the guarded ROS chirp and rosbag converter;
it does not contain a ready-to-run Jaguar-specific Isaac Lab PACE task or
simulator replay command. Add and validate that task on the PC first.

Once the Jaguar task and its fitting configuration are in place, run the
checkout's `scripts/pace/fit.py` from the Isaac Lab environment. Confirm that
the fit is restricted to FR collar, hip, and knee and that the optimizer's
parameter bounds describe Jaguar RS00 actuators. The ANYmal-D fitting bounds
and actuator configuration are not validated for Jaguar.

