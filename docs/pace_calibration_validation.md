# Jaguar PACE calibration and validation

This workflow separates actuator fitting from held-out validation. The current
hardware excitation identifies only the front-right (FR) collar, hip, and knee
while the robot is fully suspended. It does not calibrate all 12 motors or
validate ground-contact dynamics.

## 1. Collect separate robot runs

Keep the robot fully hoisted, clear of the ground, with an operator at the
emergency stop. Preserve the existing 3 Nm driver limit, Kp=40, Kd=2.5,
`+/-0.03 rad` excursion, and all script interlocks. Verify the firmware torque
limit on all motors before the test as described in
[`pace_system_identification.md`](./pace_system_identification.md).

Collect one fitting run and one held-out run. The second run reverses the chirp
frequency sweep; it is not an optimization dataset.

```bash
# Fitting run
ros2 run jaguar_control run_jaguar_chirp.py --arm --profile fit --label fit_01

# Held-out run, in a separate session
ros2 run jaguar_control run_jaguar_chirp.py --arm --profile validation --label val_01
```

When the chirp terminal prints the recorder command, run it immediately in a
separate terminal. The labels will be `pace_fit_01` and `pace_val_01`. Transfer
each rosbag with its matching `jaguar_chirp_*_commands.csv` and
`jaguar_chirp_*_metadata.json`. Do not combine the two runs.

## 2. Convert each run using its command timestamps

On the PC with the Isaac Lab environment and ROS bag support available:

```bash
cd /home/shultan/IsaacLab
./isaaclab.sh -p /home/shultan/jaguar_sim2real/scripts/pace/convert_rosbag_to_pace.py \
  /path/to/pace_fit_01_<timestamp> \
  --command-csv /path/to/jaguar_chirp_fit_01_<timestamp>_commands.csv \
  --output /home/shultan/pace-sim2real/data/nxp_jaguar/fit_01.pt

./isaaclab.sh -p /home/shultan/jaguar_sim2real/scripts/pace/convert_rosbag_to_pace.py \
  /path/to/pace_val_01_<timestamp> \
  --command-csv /path/to/jaguar_chirp_val_01_<timestamp>_commands.csv \
  --output /home/shultan/pace-sim2real/data/nxp_jaguar/validation_01.pt
```

With `--command-csv`, the converter selects only the active FR chirp using
wall-clock timestamps, aligns the recorded commands and encoder measurements,
and stores the source paths, selected time interval, and run metadata. Inspect
the reported sample count and duration; each active chirp should be about 20 s.

## 3. Fit once, then evaluate without optimizing

Run CMA-ES only on the fitting file:

```bash
cd /home/shultan/IsaacLab
./isaaclab.sh -p /home/shultan/pace-sim2real/scripts/pace/fit.py --viz none \
  --task Isaac-Pace-NXP-Jaguar-v0 --num_envs 32 \
  --data /home/shultan/pace-sim2real/data/nxp_jaguar/fit_01.pt
```

Replay the saved best candidate on the held-out run. `--evaluate_candidate`
must not be combined with fitting the validation file. Generate the nominal
uncalibrated actuator candidate and evaluate both candidates on the same held-
out file:

```bash
./isaaclab.sh -p /home/shultan/jaguar_sim2real/scripts/pace/create_pace_default_candidate.py \
  --output /home/shultan/pace-sim2real/data/nxp_jaguar/nominal_candidate.pt

./isaaclab.sh -p /home/shultan/pace-sim2real/scripts/pace/fit.py --viz none \
  --task Isaac-Pace-NXP-Jaguar-v0 --num_envs 1 \
  --data /home/shultan/pace-sim2real/data/nxp_jaguar/validation_01.pt \
  --evaluate_candidate /home/shultan/pace-sim2real/data/nxp_jaguar/nominal_candidate.pt

./isaaclab.sh -p /home/shultan/pace-sim2real/scripts/pace/fit.py --viz none \
  --task Isaac-Pace-NXP-Jaguar-v0 --num_envs 1 \
  --data /home/shultan/pace-sim2real/data/nxp_jaguar/validation_01.pt \
  --evaluate_candidate /path/to/logs/pace/nxp_jaguar/<fit-run>/best_candidate_<n>.pt

./isaaclab.sh -p /home/shultan/jaguar_sim2real/scripts/pace/analyze_pace_replay.py \
  /home/shultan/pace-sim2real/data/nxp_jaguar/nominal_candidate_replay.pt

./isaaclab.sh -p /home/shultan/jaguar_sim2real/scripts/pace/analyze_pace_replay.py \
  /path/to/best_candidate_<n>_replay.pt
```

The analysis writes per-joint position and velocity RMSE. Retain both reports
alongside the fit candidate and both datasets. The nominal candidate uses the
current Jaguar PACE defaults: 0.01 armature and zero viscous/Coulomb friction,
encoder bias, and delay. If those defaults change, update and verify
`create_pace_default_candidate.py`. The fitted candidate is provisional until
a second independent fitting run gives similar parameters and held-out errors
improve over this nominal candidate. A bias at its configured bound
requires checking encoder zero; delay must be judged against independently
measured command-to-feedback timing. If validation fails, inspect encoder zero
and timing before widening parameter bounds or increasing optimizer iterations.

The delay is quantized to the simulator's physics step. For this task, one
step is 5 ms; therefore a fractional candidate delay is not a measured
sub-step latency. Confirm command/feedback timing from timestamps and interpret
the replay's effective integer delay.

Only the FR joints receive active excitation in these profiles. Do not report
the remaining nine actuators as calibrated. A full 12-motor identification
requires separately exciting and validating those joints under the same
bounded, suspended-robot procedure.
