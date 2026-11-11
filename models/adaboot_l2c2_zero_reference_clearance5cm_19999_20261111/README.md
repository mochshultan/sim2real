# Zero-reference AdaBoot-L2C2, clearance 5 cm, model 19999

This is the MLP Jaguar policy trained through index 19999, including the
height-standing fine-tune and subsequent adaptive clearance-5cm continuation.
It is independent of the recurrent MoE-Loco policies.

Launch selection:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch jaguar_control sim2real_zero.launch.py
```

The zero launcher pins the versioned `policy_adaboot_l2c2_zero_reference_clearance5cm_19999_20261111.pt`
artifact. `models/policy.pt` and the nominal-reference launcher keep their
previous policy. The older zero-reference 5999 export and checkpoint remain
archived. No CAN driver or physical motor was run while preparing this update.

## Policy/controller contract

- Input: float32 CPU tensor `[batch,5,45]`, oldest history first. At each policy
  start, the controller repeats the first clean observation across all five
  frames and zeros previous-action features. The zero launcher selects this
  convention with `history_initialization=repeat`; legacy default is `zeros`.
- Per-frame observations: body gyro *0.25, projected gravity, command
  `[vx*2,vy*2,yaw_rate*0.25]`, joint positions relative to **mechanical zero**,
  joint velocities *0.05, previous clipped action. No standing-pose subtraction.
- Export reorders time history into term-major 225-D estimator input, estimates
  3-D velocity and concatenates it to the current 45-D policy observation.
  Frozen 48-D empirical normalization uses epsilon 0.01. There is no privileged
  input, critic, Gaussian sampling or recurrent hidden state at deployment.
- Output: deterministic policy mean clipped to `[-10,10]` **inside the export**.
  Thus the existing controller stores clipped previous actions and applies
  `joint_target = 0.25 * action`, matching the training wrapper. The legacy
  export CLI remains unbounded unless `--clip-actions 10` is requested.
- Policy frequency 50 Hz. Normal/transition Kp45/Kd4.5 are retained. Existing
  motor/joint safety limits, soft-start ramp and safe-park Kp14/Kd0.5 are
  preserved. Nominal simulated torque cap is 14 Nm; ros2_control hardware cap is
  14 Nm. Joint-limit guards, ramps and hardware timing remain in effect.

## Training and traceability

The archived `env.yaml` and `agent.yaml` are the exact final run snapshots.
Training uses base-height target 0.25 m / weight -10, no coxa-standing reward,
height-conditioned all-foot standing bonus and symmetric instantaneous foot
clearance target 0.05 m weighted by body-relative XY foot speed. These rewards
shape training; the deployment policy does not enforce a hard foot-height
constraint. Adaptive action noise was initially reset to 1.0 and then learned;
only deterministic means are deployed.

The adjacent `deployment_adaboot_l2c2_zero_reference_clearance5cm_19999_20261111.json`
manifest records the source checkpoint/run, SHA-256 hashes of model and config
files, tensor/joint conventions and CPU parity results. CPU export validation
compares against the native training actor using zero, random and stress
histories at batch sizes 1/16/64. Physical robot motion remains unverified.

Validation completed on the ROS2 base: 150 Python tests and 4 native tests
passed; jaguar_control and serial_imu builds succeeded. Installed controller, launch files and policy hashes match source.
A 10 s CPU MuJoCo checkpoint rollout at command [0.3,0,0] advanced 2.02 m,
minimum base height 0.192 m, final height 0.206 m, with finite state and no
base-below-10cm event. See `mujoco_flat_walk_10s.json`. This rollout used the
standalone nominal MuJoCo controller, not the live ROS/CAN control loop.

This ROS2 adaptation is based on main 4c57327 and retains its ros2_control
plugin, 14 Nm cap and rear-hip calibration. CLI model overrides remain supported.
The observation history option is implemented in scripts/policy_observation.py.
