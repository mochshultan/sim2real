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

The raw robot recording and CSV are included in the latest pull under
`data/pace/`; keep these files unchanged as the archival copy.

## Convert the rosbag

The converter needs ROS 2 bag Python support, NumPy, and PyTorch. On this PC,
source ROS 2 Jazzy and use the Isaac Lab Python environment:

```bash
source /opt/ros/jazzy/setup.bash
cd /home/shultan/IsaacLab
./isaaclab.sh -p /home/shultan/jaguar_sim2real/scripts/pace/convert_rosbag_to_pace.py \
  /home/shultan/jaguar_sim2real/data/pace/jaguar_chirp_20261012_145729 \
  --output /home/shultan/pace-sim2real/data/nxp_jaguar/jaguar_chirp_full.pt
```

The converter maps joints by name and writes PACE tensors in leg-by-leg order:
FR, FL, BR, BL; roll, hip, knee for each leg. It resamples to 200 Hz. Its
output includes the whole bag overlap, so trim it to the chirp interval before
fitting. In this recording, the converted bag-relative FR chirp interval is
26.0–46.0 s. The command CSV restarts its `time_s` clock for each phase; do not
use its chirp time directly as a converted-bag timestamp. Unexcited joints
cannot identify actuator parameters.

Copy the trimmed tensor to the PACE checkout at
`data/nxp_jaguar/chirp_data.pt`, matching the path configured for the custom
Jaguar task. Preserve its `time`, `dof_pos`, and `des_dof_pos` arrays and
joint order. Do not silently reorder columns or interpret commanded gains as
measured torque.

## Prepare and run the simulation pipeline

The PC checkout now includes a Jaguar-specific PACE task and simulator-side FR
chirp generator. Do not use the ANYmal-specific
`scripts/pace/data_collection.py` for Jaguar. Generate separate simulation
data with:

```bash
cd /home/shultan/IsaacLab
./isaaclab.sh -p /home/shultan/pace-sim2real/scripts/pace/data_collection_nxp_jaguar.py \
  --viz none --duration 20 \
  --output /home/shultan/pace-sim2real/data/nxp_jaguar/sim_chirp_data.pt
```

This runs the fixed-base suspended Jaguar task, holds the nine non-FR joints
at the nominal pose, applies the `+/-0.03 rad`, `0.1–0.8 Hz`, 20 s FR chirp at
50 Hz command updates, and records at the 200 Hz physics rate. It also writes
a command CSV and JSON metadata. The dataset is simulation output, not robot
measurement.

The retained robot rosbag and command CSV are now included in the pulled
checkout under `data/pace/`. The converted data contain about 46.1 s of
measurements. The stand transition and nominal hold precede the chirp; fit
only the FR chirp interval specified above.

## PACE fit

The Jaguar task is registered and its configuration imports in the Isaac Lab
environment. Fit only the three excited FR joints for this experiment; the
other nine joints are held and cannot be identified from this dataset:

```bash
cd /home/shultan/IsaacLab
./isaaclab.sh -p /home/shultan/pace-sim2real/scripts/pace/fit.py --viz none \
  --task Isaac-Pace-NXP-Jaguar-v0 --num_envs 32 \
  --data /home/shultan/pace-sim2real/data/nxp_jaguar/jaguar_chirp_full.pt \
  --start_time_s 26.0 --end_time_s 46.0
```

The task defaults to identifying the three excited FR joints. Its remaining
nine joints use a fixed PD hold and are not optimized. Use the transferred
measured dataset for identification. Use `sim_chirp_data.pt` only to check
the software pipeline; it is synthetic and cannot identify the real motors.
The ANYmal-D bounds are not Jaguar settings.

## Run recorded on this PC

On 2026-10-12, the pulled rosbag was converted to 9,215 aligned samples over
46.07 s and fitted over 26.0–46.0 s with 32 parallel environments and the
PACE limit of 200 CMA-ES iterations. The best in-sample objective was
`8.8364e-5 rad^2` at iteration 191 (per-joint RMSE `0.00543 rad`). The run
finished at iteration 200, but the final-generation score was higher and the
population spread remained large. The best candidate's FR collar and knee
encoder biases were near the configured `+0.1 rad` bound. Treat these values
as provisional fit output only; inspect the bound sensitivity and validate a
simulated replay before using them as identified Jaguar motor parameters.
Artifacts are in
`/home/shultan/pace-sim2real/logs/pace/nxp_jaguar/26_10_12_16-21-58/`.

