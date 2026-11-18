# Pion Current position deployment

The current default is Pion Current Height20 + Swing checkpoint14999, using the
single MLP actor and velocity estimator. The source run is:

```text
pion_current_baseline_height20/2026-11-17_21-05-43_pion-height20-swing-final-to15k-20261117-512env/model_14999.pt
```

`origin/main` was pulled at `d7739c6` on 2026-11-18 before this model update.
The ordinary nominal launch and its model remain separate. The previous
checkpoint9999 export is retained for explicit selection.

## Build and run

```bash
cd ~/jaguar_sim2real_ros2
bash scripts/build.sh

# Physical robot: same hardware stack as the existing launch
./scripts/sim2real_curpos

# Simulator only: same policy controller, MuJoCo instead of hardware
./scripts/sim2sim_curpos
```

Each curpos controller run automatically records timestamped observations,
estimator input/output, actor input/output, and motor commands under
`data/sessions/curpos/<run_id>/telemetry.jsonl`. These files are included by Git
when added and committed. See the [telemetry format](../data/sessions/curpos/README.md).

The wrappers source Jazzy and the local install. Python selection checks the
active Python, local `.venv`, then existing `~/IsaacLab/.venv` and
`~/IsaacLab-upgrade/.venv`. Set `JAGUAR_PYTHON=/path/to/python3` to pin another
Python3.12 with PyTorch (and MuJoCo/Viser for simulation). Keep ROS dependencies
available by sourcing Jazzy; do not prepend the entire system dist-packages to a
venv's PYTHONPATH, since that can mix incompatible compiled dependencies.

For a new environment, install the requirements into the selected Python:

```bash
uv venv --python /usr/bin/python3 .venv
uv pip install --python .venv/bin/python -r requirements-curpos.txt
```

The browser simulator defaults to `0.0.0.0:8080` and can run on CPU. Connect from
another machine at `http://<simulation-host-ip>:8080`. Sliders control forward,
lateral and yaw velocities. The model starts in its physical standing pose;
A/Stand and B/Policy controls are the same commands used by the real controller.
Auto-walk performs that sequence by default; use `auto_walk:=false` for manual
arming. A zero command while in WALK still evaluates the learned standing policy.

```bash
# Headless verification with the actual ROS controller
./scripts/sim2sim_curpos viz:=none duration:=20.0 command_x:=0.3 report:=/tmp/pion-curpos.json
# Different browser port; another simulation domain if needed
./scripts/sim2sim_curpos port:=8081 domain_id:=72
```

Do not run two simulations in the same DDS domain. Simulation defaults to71;
physical launch inherits the existing domain1 default. Teleop/CLI publishers for
simulation must also use ROS_DOMAIN_ID=71. The launch ends when its simulator
node exits. It never launches the physical CAN/hardware or serial-IMU stack.

## Shared controller contract

| Item | Value |
|---|---|
| Checkpoint/export | `model_pion_current_height20_swing_native_14999_20261118.pt` / matching `policy_...pt` |
| Actor | MLP48 ->512 ->256 ->128 ->12 |
| Velocity estimator |225 ->128 ->64 ->3 |
| Export input | Time-major history `[B,5,45]`; internally repacked term-major225 |
| History cold start | Repeat first observation; previous actions zero |
| Joint observations | Calibrated q, zero reference |
| Action | `q_target=q_measured+0.25*clip(action,-10,10)` |
| Anchor | Once per20ms policy step; target held during faster PD updates |
| Physical standing | Collar0, hip-1.25, knee+1.25rad |
| Gains | Kp45, Kd4.5 |
| Motor torque cap |14 Nm through the driver/firmware; sim uses the saved linear32V torque-speed approximation |
| Training base height |20cm from the base-link origin perpendicular to the local ground plane, weight-100 |
| Training clearance |Contact-derived swing arc 0 ->3cm ->0, weight-1; hip alignment near the swing apex, weight-0.1 |
| Runtime inference | Estimator-only; no privileged critic or simulator velocity input |

The dedicated runtime is `scripts/pion_controller.py`, with its native contract
in `scripts/pion_policy.py` and configuration in `config/sim2real_pion.yaml`.
Policy state, history and actions use FL/FR/BL/BR grouped collar/hip/knee.
The native TorchScript contains no Jaguar permutation. Named joint feedback and
commands provide the mapping to the existing CAN hardware. The controller reuses
the existing stop/state machine, but applies no action EMA or WALK ramp. Bounded IMU leveling is enabled
only during STANDUP and STAND_HOLD, including return from WALK. Sim2sim launches this same Pion node.

`pion_mujoco_plant.py` contains only plant dynamics and impedance tracking.
`sim2sim_curpos_bridge.py` translates `/joint_commands`, `/joint_states`, `/imu/data`
and the mode/velocity topics. It emulates the configured IMU mounting orientation
so the controller performs its normal body-frame correction once. Simulated
feedback is already in calibrated joint coordinates; physical motor offsets are
not added a second time.

The export includes a contract flag `action_reference=current`; the controller
rejects absolute-action exports when launched as current-position. The baseline
and model reference are distinct from the standing pose. Zero action holds the
measured position at the policy sampling instant; it does not request0rad or a previous
target. Existing absolute-zero exports can still be used explicitly with the
shared main launch and `policy_reference:=zero`, but are not the curpos default.

## Recording and limits

Physical curpos sessions are saved as `sim2real_curpos_*.jsonl`, and simulator
sessions as `sim2sim_curpos_*.jsonl`, under
`~/.ros/jaguar_control/sessions/`. They include policy history/action, calibrated
state and motor targets. The header names the policy and current reference.
The deployment manifest contains source/export hashes and parity results.
Simulation reports are offline measurements, not hardware performance results.

The Pion MJCF and its meshes are included under `sim2sim/pion_current/`; no
external IsaacLab asset path is needed at runtime. They originate from the
Pion URDF/training-collider MuJoCo conversion, with passive joint armature0.01.
MuJoCo contacts, solver, nominal material/inertial parameters and communication
timing differ from training and hardware. A0.1s bridge command timeout makes the
plant passive; it does not reproduce all firmware fault latches. The32V curve is
an approximation rather than a measured motor model. Sim2sim is useful for
checking the actual deployment controller against a physical simulation, but
cannot establish numerical equivalence or guarantee real-robot stability.

## Current Height20 model update on 2026-11-18

The historical Height20 checkpoint was trained without target clipping. The
training workspace now provides separate `Height20-Swing-TargetLimits` and
`LowSpeed-Gait-TargetLimits` variants. They keep the current-position action
equation and clip its resulting targets to the intersection below. Running
training and historical tasks are not modified.

| Joint group | Controller guard (rad) | CAN driver guard (rad) | Training variant (rad) |
|---|---|---|---|
| Collar | [-0.5,0.5] | [-0.4,0.4] | [-0.4,0.4] |
| Hip | [-2.5,0.2] | [-3.14,3.14] | [-2.5,0.2] |
| Knee | [-0.25,2.5] | [-0.1,2.8] | [-0.1,2.5] |

Both hardware guards are retained. The MuJoCo plant now also applies the CAN
guard after the shared controller guard, rather than omitting this second
clipping stage. This changes simulator execution, not the policy export.
See `data/sessions/curpos/README.md` for per-motor CAN reception-age telemetry.

Both curpos launches select `policy_pion_current_height20_swing_native_14999_20261118.pt`.
The controller verifies the paired `deployment_...json` filename and SHA-256.
The previous native checkpoint9999 remains compatible when selected explicitly.
Strict loading into the training actor and 128 randomized five-frame histories
gave a maximum export error of 0.0. Physical hardware motion has not been tested
with this new model. The checks and source hashes are stored in its manifest.

## Historical native Pion update on 2026-11-17

`sim2real_curpos` selected `pion_controller.py`, `sim2real_pion.yaml`, and
`policy_pion_current_native_9999_20261117.pt`. The selected export hash and contract
are checked at startup. The main nominal launch keeps the Jaguar controller.

- Sampling: 50 Hz policy, 200 Hz hardware; latest-sample joint feedback QoS.
- Proprioception: 45D plus 3D estimated velocity, five-frame estimator history.
- Gains: Kp=45, Kd=4.5; action scale=0.25; action clip=10.
- Standing/reset reference: collars=0, hips=-1.25, knees=1.25 rad.
- WALK target: sampled measured position plus scaled action, held between updates.
- No extra WALK ramp or action EMA. Standing IMU trim is active outside WALK,
  during STANDUP/STAND_HOLD. Native Pion collar signs are FL-, FR+, BL-, BR+;
  hip correction is bounded at 0.15 rad, collar at 0.12 rad, knees unchanged.
- Torque cap=14 Nm through the driver/firmware, as requested. The historical training randomization was 12.6--13.5 Nm.
- Hardware mechanical limits and emergency guards remain active. Session samples
  include targets before limiting and a per-joint clipping mask.

The training delay is randomized from 0--15 ms; no artificial delay is added to
hardware. Actual transport delay must be measured. Noise, friction and strength
randomization describe a training distribution, not values to inject into real
sensor feedback. Physical motor response and geometry still need measurement.

## Native validation on 2026-11-17

- 168 Python tests and four C++ tests passed; ROS build passed.
- Native export and the previously verified export agree exactly when the same
  physically named histories are expressed in their respective joint orders.
- ROS/MuJoCo stand, forward and backward smoke runs entered WALK and exited cleanly (recorded with the earlier 13.5 Nm deployment cap).
- Session replay errors were at most 1.2e-6 (batched floating-point inference).
  All recorded pre-limit targets matched sampled q + 0.25 action exactly.
- No controller target clipping occurred in these three short runs.
- No CAN hardware was launched or moved. These checks do not establish hardware
  stability or measured actuator equivalence.

## Earlier adapter validation on2026-11-17

- Export/native actor equality: maximum absolute difference0.0 for the recorded
  zero/random/stress cases at batch sizes1/16/64.
- Actual ROS controller + MuJoCo: zero command, +0.3m/s forward, -0.3m/s backward,
  and a Viser run all entered WALK. Viser HTTP returned200.
- Recorded policy histories reproduced the recorded actions exactly. After the
  walk ramp, motor targets matched clipped(q_measured+0.25*action) exactly.
-166 Python tests and4 C++ tests passed; ROS2 build completed. Native deprecation
  warnings remain for TorchScript and ROS C++ accessors.
- No physical hardware was started or moved. These short simulator runs do not
  establish long-horizon convergence or real-robot stability.
