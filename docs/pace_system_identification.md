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

1. For passive recording, use the normal known-good launch. For the active
   chirp, use the isolated driver procedure below and do not start the controller.
2. Hoist and secure the robot fully off the ground so all feet remain in the
   air throughout the pose transition and chirp. Clear the workspace, keep the
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

The recorder does **not** create an excitation. A guarded Jaguar pilot chirp is
available in `scripts/pace/run_jaguar_chirp.py`; it is intended only for a
fully suspended robot, and it refuses to arm unless a separate driver reports
a configured 3 Nm torque limit, fresh feedback, a passive starting state, and no
other motor-command publisher. It starts in preview mode; physical motion
requires `--arm` and starts automatically five seconds after preflight passes
(`Ctrl+C` cancels during the countdown). It first moves all 12 joints to the
configured standing pose over 18 seconds and holds them there. It then excites
only the front-right (FR) collar, hip, and knee at `+/-0.03 rad` by default
(hard cap `+/-0.03 rad`), 0.1-0.8 Hz
over 20 s, with a 0.15 rad/s target-speed guard, Kp=40, and Kd=2.5 on all
joints; the other nine actively hold
the standing pose. Keep the robot fully suspended. Torque, velocity,
tracking-error, temperature, feedback-freshness, and joint-limit stops remain
active. The command trajectory, including the transition, is saved as CSV
under `data/pace/`, with wall-clock timestamps and a JSON run manifest. Use
`--profile fit` for an ascending 0.1-0.8 Hz sweep and
`--profile validation` for a descending 0.8-0.1 Hz sweep. Both profiles use
the same excursion, gains, duration, command rate, and safety guards; validation
data must be kept separate from fitting data.

Startup velocity must be below 0.10 rad/s for all joints. Before motion, every
joint must also be no more than 0.20 rad outside the CAN driver's configured
position range; otherwise the driver's SAFE_PARK interlock would trigger when
active commands begin. Runtime torque, temperature, and high-velocity stops
monitor all 12 joints. The transition checks tracking on all joints; during
the chirp, tracking and position-margin checks apply to all 12 held joints.

The command gains are Kp/Kd = 40.0/2.5, matching the tuned Jaguar sim2real
controller gains; feed-forward torque is zero. Configure the Jaguar
PACE actuator's `stiffness` and `damping` to 40.0 and 2.5 respectively for
simulator replay. This first single-leg recording excites only three joints;
fit only those joints (or mask the other nine in the fitting loss), since
unexcited joints do not provide data to identify their parameters.
The driver reports the torque limit it configured, but the current code does
not read the value back from each motor. Before starting this procedure, verify
that parameter `0x700B` reads 3 Nm on all 12 RS00 motors with the vendor tool.
Close the vendor tool before launching the CAN driver; do not arm if readback
cannot be checked.

Rebuild the package after updating the workspace:

```bash
cd /home/erc/sim2real
colcon build --packages-select jaguar_control --symlink-install
source /opt/ros/humble/setup.bash
source install/setup.bash
```

For the fitting recording, start the chirp process first. It checks for competing publishers without
sending commands, then opens a zero-gain passive heartbeat and waits for the
hardware driver. Wait until it prints “Passive zero-gain heartbeat starts
now”. The robot remains passive until the automatic five-second countdown
after preflight:

```bash
cd /home/erc/sim2real
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run jaguar_control run_jaguar_chirp.py --arm --profile fit --label fit_01
```

Then, in another terminal start only the C++ CAN driver, with no controller or
other command publisher:

```bash
cd /home/erc/sim2real
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run jaguar_control robstride_can_node --ros-args \
  --params-file config/sim2real.yaml -p control_torque_limit_nm:=3.0
```

The driver publishes its configured torque limit on
`/jaguar/control_torque_limit_nm`; the chirp aborts if it is not 3 Nm. After
the all-joint stand transition, the chirp terminal says “Nominal standing pose
reached and held. Start rosbag now.” Start rosbag in a third terminal then;
the FR chirp begins automatically after a five-second hold:

```bash
bash scripts/pace/record_jaguar_joint_data.sh pace_fit_01
```

The chirp terminal prints the exact bag-recorder command after the stand pose
is reached. Run that command immediately in the recorder terminal. Repeat the
same procedure for held-out data, using `--profile validation --label val_01`
and the printed matching bag label. Do not fit the validation recording. Keep
each rosbag, command CSV, and JSON manifest together; the converter uses the
CSV's wall-clock chirp timestamps to exclude the transition and hold periods.

Do not run `sim2real.launch.py` or start the Jaguar controller during this
test: the chirp checks for competing publishers and will stop if one appears.
On a safety threshold violation it requests the driver's latched emergency
stop. After a normal completion it sends passive zero-gain commands; because
the driver watchdog is 100 ms, closing the chirp process then causes a latched
driver stop. The body must remain secured on its support throughout, including
after any stop.

The upstream PACE ANYmal-D chirp is not used on Jaguar; its joint offsets and
amplitudes do not match Jaguar limits. The Jaguar pilot is a first bounded
measurement, not a validated full identification trajectory. Inspect the bag,
CSV, and limits before using it for fitting. Do not fit ordinary walking or
standing bags using the current fixed-base PACE task.

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
locomotion data suitable for system identification. A guarded Jaguar-specific
active excitation is available as a pilot; its measured response and safe
operating range still need validation before using the data for fitting.

Keep the controller/gain path identical between real measurement and
simulator replay. PACE's provided ANYmal-D actuator configuration is not a
Jaguar QDD configuration; validate the actuator model and parameter bounds
before fitting.

The default Jaguar ROS hardware loop publishes encoder feedback at 200 Hz;
the locomotion controller's joint targets are typically published at 50 Hz.
Inspect recorded rates and timestamp jitter before choosing a replay timestep.

## Current limits

- The Jaguar chirp is a guarded pilot only. Its frequency, amplitude, gains,
  and multi-joint phase pattern still need validation against the Jaguar PACE
  model before treating the measurements as a final identification dataset.
- The PACE repository's reference task, actuator, joint names, and fitting
  bounds are for ANYmal D. They must be ported and checked for Jaguar before
  running `scripts/pace/fit.py`.
- Isaac Sim 6.1 is installed locally, while PACE's current classifiers list
  Isaac Sim 4.5/5.0. Import and a small Jaguar environment smoke test are still
  required before relying on PACE with the local simulator.

For the separate PC workflow, including transfer of the suspended-robot data,
simulation replay requirements, and PACE fitting prerequisites, see
[`pace_pc_training_notes.md`](./pace_pc_training_notes.md).
For the split fit/held-out validation procedure, see
[`pace_calibration_validation.md`](./pace_calibration_validation.md).
