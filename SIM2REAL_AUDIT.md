# Sim2real Repository Audit

Date: 2026-09-07. Baseline: local branch `cpp`, commit `1952f8f`, including the existing uncommitted changes.

This is a review, not a motor-control fix. No application source, configuration, calibration, Git remote, or hardware state was changed during this audit. Findings describe the current checkout; some regressions were introduced by the earlier assistant edits and partial rollback in this conversation.

## Scope and Verification

Reviewed the CAN drivers, controller and diagnostic workflows, calibration tools, teleoperation, serial IMU decoder, simulation entry points, model interfaces, build/install rules, configuration, and documentation. Mechanical geometry and binary mesh correctness were not validated. This report does not claim to identify every possible defect.

- All 31 tracked Python files pass syntax compilation. This does not verify runtime API compatibility.
- An AST call-site check found 14 remaining calls with unsupported `timeout` keywords across four Python tools.
- A standalone C++17 syntax check of the hardware-manager header fails at two undefined `wrapAngle` calls.
- Offline Python probes reproduced incorrect CAN-frame acceptance, continued active commands after rejected feedback, missing startup gating, joystick velocity latching, and delayed passive-mode handling.
- A C probe using UndefinedBehaviorSanitizer reproduced an out-of-bounds IMU item-array write; additional probes reproduced acceptance of truncated quaternion data and the bad-CRC publish condition.
- The bundled TorchScript model accepts `(1, 5, 45)` and returns finite `(1, 12)` output for a zero-input smoke test. `(1, 45)` and `(1, 48)` fail.
- Tracked shell, YAML, and XML/URDF files pass syntax checks. Syntax does not establish launch compatibility or operational correctness.
- Pyflakes reports 26 unused imports and 13 unused assignments. Its `Tensor`/`Tuple` findings in `isaacgym_torch_utils.py` are type-comment issues, not demonstrated Python import failures.
- No CAN or serial devices were opened, motors enabled, ROS control nodes launched, or firmware/zero settings changed. No full colcon build or graphical simulation was run. MuJoCo is unavailable in this environment.

Local offline probes are in `/tmp/sim2real-audit-H01q67/`. These are diagnostic fixtures, not a maintained regression suite.

Priority: P1 = address before powered motion or normal deployment; P2 = functional/reliability defect; P3 = maintenance/documentation defect.

## P1 Findings

### R01. The C++ driver currently does not compile

References: `include/jaguar_control/robstride_hardware_manager.hpp:243`, `:319`; `include/jaguar_control/robstride_protocol.hpp`.

The manager still calls `wrapAngle`, but the earlier rollback removed its definition from the protocol header. The C++17 compiler reports both calls as undefined. A Python-only compile check could not catch this. Restore a coherent implementation and test the manager, not just individual Python files. Do not blindly restore modulo wrapping; see R05.

### R02. The partial API rollback still breaks four Python tools

References: `scripts/robstride_motor_lib.py:348`; `scripts/can_hardware_node.py:202`; `scripts/calibrate_sit_zero.py:325`; `scripts/calibrate_stand_pose.py:329`; `scripts/robstride_motor_test.py:68`.

`send_control_command` no longer accepts `timeout`, but 14 calls still pass it. The Python CAN worker fails during startup; calibrators swallow the resulting TypeError and retain initial readings; motor tests fail on their first control call. Only `test_sit_stand.py` was repaired in the previous turn. Fix the interface and all consumers together, with runtime call-contract checks.

### R03. CAN replies are not associated reliably with the requested motor or message type

References: `scripts/robstride_motor_lib.py:260`, `:275`; `scripts/check_joints.py:62`; `include/jaguar_control/robstride_protocol.hpp:227`.

The Python library returns the first queued frame and parses its bytes as telemetry without checking frame format, source ID, communication type, or payload length. Callers assign the result to the requested joint even when the returned motor ID differs. A controller for ID 1 accepted an ID 6 reply; it also accepted a type-0 payload as a position. A one-byte payload raises IndexError. C++ additionally accepts type-1 command data as feedback. Delayed replies, queued discovery traffic, or another process can therefore contaminate positions. Demultiplex validated replies before updating state.

### R04. Stale and missing feedback is presented as fresh state

References: `include/jaguar_control/robstride_hardware_manager.hpp:44`, `:238`, `:324`; `src/robstride_can_node.cpp:254`; `scripts/can_hardware_node.py:267`; `scripts/nxp_jaguar_controller.py:355`, `:452`.

The C++ manager stores feedback timestamps but never expires them. Its watchdog monitors incoming commands, not feedback. Both hardware publishers stamp cached/default values with the current ROS time. The RL controller checks only sticky `imu_received` and `joints_received` booleans, so an IMU or CAN feedback outage after startup does not stop continuing commands. Track freshness per joint and sensor, and propagate invalidity rather than refreshing cached measurements' apparent age.

### R05. The added noise filter does not prevent a multi-turn corrective movement

References: `scripts/robstride_motor_lib.py:288`, `:348`; `scripts/test_sit_stand.py:433`, `:468`; `include/jaguar_control/robstride_hardware_manager.hpp:243`, `:319`; `scripts/nxp_jaguar_controller.py:380`, `:600`.

Rejecting a 12-rad reading leaves cached state intact and does not inhibit commands. The offline probe rejected that sample and then successfully generated another command with positive Kp. The absolute 2.5-rad threshold also rejects measurements without considering joint configuration or startup encoder reference; the jump test compares unscaled position against scaled cached position. Separately, modulo tracking errors can hide full-turn differences: a 4-pi error becomes zero. Host-side wrapping does not establish the motor controller's internal target coordinate. Resolve the raw encoder reference and stop/inhibit active control on invalid or stale feedback before resuming. Raw captures are still needed to distinguish a protocol-decoding error from encoder reference behavior on this robot.

### R06. The Python hardware fallback lacks a command watchdog and emergency-stop subscription

References: `scripts/can_hardware_node.py:87`, `:145`, `:220`, `:239`; `src/robstride_can_node.cpp:76`.

Once `cmd.enabled` becomes true, the Python driver continues using the last commands indefinitely. It has neither command-age enforcement nor a `/jaguar/emergency_stop` subscriber. Therefore selecting the documented Python fallback changes stop behavior substantially. This defect becomes operational once R02 is repaired. Both drivers need the same stop and timeout contract.

### R07. Stand/sit activation does not require verified telemetry

References: `scripts/test_sit_stand.py:118`, `:294`, `:430`; `scripts/calibrate_sit_zero.py:316`, `:336`; `scripts/calibrate_stand_pose.py:316`.

The tester starts with zero positions and 25 C temperatures, reads only once before proceeding, and permits transitions without a validity/freshness check. A direct probe of `start_transition` with untouched initial state enters `TRANSITIONING` and sets `is_passive=False`. Initialization also replaces measured positions with wrapped/clamped values, concealing an out-of-range initial posture. Require fresh, valid measurements for every participating motor before activation; keep measured state separate from constrained targets.

### R08. Passive requests and shutdown can race with active CAN commands

References: `scripts/test_sit_stand.py:449`, `:541`, `:702`; `scripts/robstride_motor_lib.py:260`; `scripts/can_hardware_node.py:295`.

The tester snapshots `is_passive` once for a whole 12-motor cycle. A stop request arriving during the first send does not affect the remaining sends. The offline probe reproduced 11 further positive-gain commands after setting passive mode. Each receive can wait one second, so an unresponsive bus substantially extends response time. Shutdown also starts sending on shared sockets without first coordinating completion of the worker. Serialize communication ownership, make stop state interrupt subsequent active sends, and bound transaction times.

### R09. Releasing or disconnecting the joystick can leave walking velocity latched

References: `scripts/keyboard_teleop.py:120`, `:147`, `:200`; `scripts/gamepad_reader.py:247`.

The joystick callbacks update velocity only while an axis is outside the deadzone. Returning all sticks to center does not write zeros. The publish timer then repeatedly sends the previous velocity, keeping the controller's command watchdog alive. The probe reproduced `vx=1.0` after release. The disconnection path marks the gamepad disconnected without clearing teleop's latched command. Define input ownership and publish zero on release/disconnection of the active input source.

### R10. The stop-all script bypasses motor cleanup and misses the default driver

References: `scripts/stop_all.sh:10`, `:32`, `:55`.

`pkill -9` prevents Python finally/atexit cleanup. The target list omits `robstride_can_node`, both calibration tools, and the motor test. It then brings CAN interfaces down without sending confirmed stop commands. This does not establish that actuators have stopped, and the C++ process can remain running. Stop through the owning driver, wait for shutdown, then close interfaces; process killing is not an actuator-stop acknowledgement.

### R11. Fault handling can initiate additional powered motion

References: `scripts/nxp_jaguar_controller.py:295`, `:471`, `:485`, `:538`, `:599`.

Over-torque, excessive tilt, and tracking errors enter `SAFE_SHUTDOWN`, which commands a three-second trajectory with positive transition gains before cutoff. Tracking checks exclude that state, and over-torque checks also exclude `STAND_HOLD`. A jammed actuator can therefore receive further powered movement after the fault. Use an immediate inhibited state for motion/feedback faults and a separately requested controlled parking trajectory when its prerequisites are valid.

### R12. The IMU parser writes beyond fixed arrays and accepts incomplete items

References: `serial_imu/src/ch_serial.c:88`, `:100`, `:180`; `serial_imu/src/ch_serial.h:46`.

`item_code[8]` is indexed with an unchecked growing counter. Gateway node counts are not bounded by `MAX_NODE_SIZE`, and individual item sizes are not checked against remaining payload bytes. UBSan reproduced index 8 out of bounds using nine valid ID items. A CRC-valid one-byte quaternion item was accepted despite lacking quaternion data. Check item count, node count, and payload bounds before every decode/write.

### R13. CRC failures cause stale IMU values to be republished

References: `serial_imu/src/serial_port.cpp:141`; `serial_imu/src/ch_serial.c:214`; `serial_imu/src/ch_serial.h:56`.

`ch_serial_input` returns -1 on a checksum error, but the publisher checks `if (rev && raw_.nimu > 0)`. In C++, -1 is true. After a valid packet, a bad packet therefore passes the publish condition with prior sensor values and a new timestamp. The C probe reproduced exactly this condition. Require explicit decode success and valid fields.

### R14. A Git credential is embedded in the local remote URL

Reference: `.git/config`, remote `origin`.

A read-only parsed check confirmed an embedded password/token in the remote URL. The token value was not printed during this audit, but an earlier turn displayed the remote verbatim. Rotate/revoke the exposed credential and store its replacement in a credential manager; use a remote URL without credentials. The audit did not modify the remote or test the token's validity.

## P2 Findings

### R15. C++ shared command data has unsynchronized accesses

References: `include/jaguar_control/robstride_hardware_manager.hpp:175`, `:185`, `:230`, `:240`.

The ROS callback writes `last_command_time_` and `commands_` under `cmd_mutex_`; the communication loop reads the timestamp without that mutex and reads commands under only `state_mutex_`. Atomic mode flags do not protect these non-atomic fields. These are C++ data races. Read a coherent command snapshot under the command mutex and define consistent locking for watchdog evaluation.

### R16. Partial or malformed command messages can activate stale joints or access past arrays

References: `src/robstride_can_node.cpp:150`; `include/jaguar_control/robstride_hardware_manager.hpp:184`; `scripts/can_hardware_node.py:104`, `:145`.

Updating one C++ joint refreshes a global watchdog and exits passive mode for the entire robot, allowing other joints' old commands to resume. Python sets `enabled=True` even for empty/unrecognized messages. In the C++ named-message path, more than 12 recognized/repeated names with a 24-element effort array can index beyond `effort[12+i]`. Validate message dimensions, unique names, and completeness before atomically committing a command, or define explicit per-joint freshness semantics.

### R17. Python enable framing does not match the documented private protocol

References: `scripts/robstride_motor_lib.py:304`; `include/jaguar_control/robstride_protocol.hpp:145`.

The Python enable command has an empty payload (DLC 0), while C++ and the manufacturer example use eight bytes. The earlier advice to use `FF ... FC/FD` mixed the separate standard-frame MIT protocol with extended-frame private commands; those earlier edits have been rolled back. The manufacturer's private protocol uses type 3 for enable, type 4 for stop, and type 2 for feedback. Verify exact framing against the installed firmware with offline frame tests and captured replies before changing hardware behavior. See the [RS00 manufacturer manual, private-protocol section](https://files.seeedstudio.com/products/RobStride/Product%20Literature/RS00/RS00User%20Manual251112.pdf).

### R18. Motor initialization reports success without confirming successful configuration

References: `include/jaguar_control/robstride_hardware_manager.hpp:112`, `:123`, `:156`; `scripts/check_joints.py:43`; `scripts/can_hardware_node.py:169`.

The C++ manager accepts one failed bus, ignores send failures, and reports all motors initialized without checking feedback from all 12 motors. Python enables before configuring the mode and passive command, and the fallback formats potentially None feedback as a float before handling failure. Initialization can therefore report readiness with missing motors or fail partway through. Readiness should be a verified state; initialization failures must prevent active commands.

### R19. The Isaac simulator does not supply the bundled policy's input contract

Reference: `sim2sim/sim2sim_isaaclab.py:278`.

It writes commands to `policy_obs[:,9:12]` and calls the policy directly without constructing the required history. The shipped model requires `(N,5,45)`. With that shape, index 9 on axis 1 is invalid; with a 2D observation, model inference fails. Moreover, command fields in the repository's 45D format are 6:9, not 9:12. Implement one observation adapter shared with deployment and verify the tensor contract before stepping the simulator.

### R20. Simulation and hardware use different nominal poses for the same policy

References: `sim2sim/observation_builder.py:30`, `:69`; `scripts/nxp_jaguar_controller.py:55`; `scripts/test_sit_stand.py:45`.

MuJoCo loads a developer-specific external Isaac configuration, falling back to hip angles -1.55/-1.45 and knee angles 1.4/1.35. Deployment uses -1.40/-1.30 and 1.30/1.40; the stand tester has another pose. The simulation difference changes both relative observations and action baselines. Stand-test poses may intentionally differ, but the policy's nominal pose must match its training/export metadata. Package that metadata with the model rather than silently selecting a machine-dependent default.

### R21. Published joint names do not match the displayed URDF

References: `scripts/parameters.py:50`; `src/robstride_can_node.cpp:247`; `models/urdf/nxp_jaguar.urdf:114`; `launch/display.launch.py:28`.

Hardware publishes names such as `FR_collar_joint`, while the displayed URDF expects `Fr_roll_joint`. A structured comparison found zero matching names among the 12 moving joints. The controller remaps names internally, but robot_state_publisher does not receive that remapping. Align the published naming contract or provide an explicit display adapter.

### R22. Runtime defaults use source-tree paths that fail after normal installation

References: `scripts/parameters.py:88`; `scripts/nxp_jaguar_controller.py:205`; `CMakeLists.txt:53`, `:75`.

The Python helper looks for `../config` relative to scripts, and the controller's direct-run fallback looks for `../models`. CMake installs scripts in `lib/jaguar_control` and assets in `share/jaguar_control`, so those relative defaults fail in a normal non-symlink installation. Missing config silently falls back to built-in gains. The simulator's installed location also cannot find the sibling `scripts` directory it inserts into sys.path. Resolve package-share resources and install importable Python helpers consistently.

### R23. A sourced build script can report success after build failure

Reference: `scripts/build.sh:7`, `:41`, `:55`.

The script enables `set -e` only when executed, while its suggested `source scripts/build.sh` path does not explicitly check colcon's return status. In a normal interactive shell, it proceeds to source an old installation and prints success even when compilation failed. Propagate the build failure before sourcing an overlay or printing success.

### R24. Calibration output changes unrelated safety limits and some printed constants are unused

References: `scripts/calibrate_sit_zero.py:689`; `scripts/calibrate_stand_pose.py:727`, `:739`.

The sitting calibrator prints replacement C++ joint configurations that widen hip limits to +/-3.14 and knee limits to -0.10..2.80, overwriting the current limits while ostensibly updating offsets. The standing calibrator tells users to insert `DEFAULT_ANGLE` into the tester, which uses `STAND_POSE`, and `DEFAULT_JOINT_POS` into the observation builder, which uses `DEFAULT_JOINT_POS_ISAAC`. Those edits would not update the advertised targets. Calibration export should update only named calibration fields in a shared schema.

### R25. The remote Xbox sender has no matching receiver in this repository

References: `scripts/remote_xbox_forwarder.py:71`, `:118`; `scripts/keyboard_teleop.py:66`.

The forwarder sends JSON UDP packets to port 9876. Repository-wide searches found no UDP receive path or bridge for that format; robot-side teleop subscribes to ROS Joy topics instead. The documented standalone remote workflow is incomplete unless an external receiver is installed. Supply/document the receiver and its disconnect behavior, or use the ROS transport consistently.

### R26. IMU reconnect and fallback-port selection are incomplete

References: `serial_imu/src/serial_port.cpp:116`, `:132`; `scripts/bringup_imu.sh:22`, `:77`.

The serial node retries only when `fd_ < 0`, but a read error/disconnection leaves the descriptor unchanged, so the reconnect branch may never run. The shell script selects and prints an alternate `IMU_PORT`, then launches the node without passing it; the node still defaults to `/dev/ttyUSB0`. Handle terminal read errors and forward the selected port as a ROS parameter.

### R27. The display launch ignores its model argument

Reference: `launch/display.launch.py:14`, `:26`.

The launch declares `model` but reads `default_urdf` directly. Passing `model:=...` does not change robot_description. Resolve the launch substitution at execution time and load the requested file.

### R28. ROS 1 entry points are installed/documented as part of a ROS 2 package

References: `scripts/jaguar_main.py:15`, `:19`, `:356`, `:498`; `launch/fast_lio.launch:3`; `launch/README.md:23`; `CMakeLists.txt:62`.

The legacy driver imports `rospy` and `jaguar.msg`, and uses missing `P.SIM_HZ` and `P.MOTOR_OFFSET_THRE`. Several sensor launch files use ROS 1 syntax and the old `jaguar` package name. Required legacy helper modules are also omitted from the Python install list. These are not runnable ROS 2 fallback paths as documented. Clearly isolate/archive legacy code or port its dependencies and launch interfaces.

### R29. Held joystick buttons can restart transitions and interrupt shutdown

Reference: `scripts/nxp_jaguar_controller.py:411`.

The SIT branch is level-triggered and permits any state except `SIT_HOLD`. Repeated messages while X is held reset the transition start time even when already in `SITDOWN`. The same branch allows `SAFE_SHUTDOWN` to be overwritten. Apply edge-triggered mode requests and a state transition table that preserves fault-state priority.

### R30. Controller readiness accepts malformed joint messages

Reference: `scripts/nxp_jaguar_controller.py:355`, `:381`.

`joints_received` becomes true even for an empty or entirely unrecognized JointState message. Recognized names count as matched without requiring position data. Partial and non-finite measurements can therefore satisfy startup readiness while other joints retain defaults. Validate all required fields and track readiness per joint, independently from merely receiving a ROS message.

## Generated-Code and Documentation Problems

These findings concern observable quality, not a claim about who or what authored the code.

- **Unsupported safety assurances:** `check_joints.py:97`, `test_sit_stand.py:713`, and calibrator initialization/shutdown messages claim safe/passive/connected states unconditionally. They cannot establish those states when exceptions, send failures, or missing feedback are ignored.
- **Invented architecture descriptions:** `README.md:7`, `:28`, `:38`, `:56`, and `:60` describe 48D inputs, `/robot_joint_states`, `/joint_command`, two CAN threads, and hardware-manager quintic trajectories. Current code uses 5x45 inputs, `/joint_states`, `/joint_commands`, and one C++ communication thread. The manager contains no quintic startup trajectory. The `SCHED_FIFO` request can also fail; the code does not establish guaranteed sub-millisecond latency.
- **Inaccurate trajectory claims:** `test_sit_stand.py:11` and `:512` describe cosine interpolation as zero-jerk. Its endpoint acceleration is nonzero, so connecting it to a stationary hold does not provide the claimed boundary smoothness.
- **Duplicated state machines and constants:** The two calibrators and stand tester copy substantial control, terminal, safety, and rendering logic. The same unsupported API spread across them. Gains, limits, nominal poses, offsets, and name mappings have several independent definitions; the calibration exporter demonstrates actual drift, not merely stylistic duplication.
- **Misleading save actions:** Calibrator save commands print code snippets; they do not persist a verified calibration record. The UI and instructions should say exactly what is recorded and where.
- **Incomplete simulator selectors:** `sim2sim/sim2sim_mujoco.py:74` accepts `load_run` and `task` but does not use them to select the policy. Failed checkpoint export still returns the export path, potentially selecting an old file. Explicit policy requests should fail clearly rather than silently falling through to unrelated models.
- **Documentation drift:** `launch/README.md:17` documents `with_joy=false`, while the launch defaults to true. Several READMEs use machine-specific file URLs. Legacy names and copy-pasted gains contradict the current configuration.
- **Low-value clutter:** 26 unused imports, 13 unused assignments, redundant f-strings, large decorative terminal output, and commented-out legacy blocks increase review noise. Cleanup should follow behavior fixes and should not replace instrumentation or fault reporting.
- **Missing automated regression coverage:** No maintained test directory or CI pipeline was found, and CMake declares no tests. Files named `test_sit_stand.py` and `robstride_motor_test.py` are hardware-driving utilities, not safe unit tests. Earlier compile-only checks missed the API rollback and cross-file C++ breakage.

## Corrections to Earlier Advice

The earlier statement that the motor-ID bit position was the root cause was unsupported: the inspected Python code already extracted bits 15..8. The later `FF ... FC/FD` recommendation confused two protocols. Equality of local HEAD and a cached `origin/cpp` reference did not establish that the remote had been freshly fetched. The earlier assurances that all motors would become online or that the filter prevented two-turn movement were not justified by syntax checks or synthetic parser tests.

## Recommended Repair Order

1. Rotate the exposed credential. Establish a reproducible baseline and regression checks for both C++ and Python before further deployment.
2. Repair the C++ build and Python call contract together; validate framing and route feedback by bus, motor, and message type.
3. Implement telemetry freshness, startup gating, and a consistent latched fault/stop path with bounded communication time.
4. Resolve raw motor/joint coordinate handling with captured feedback and firmware documentation; verify that target and measured position use the same reference.
5. Repair joystick release/disconnection, shutdown ordering, and IMU parser validation.
6. Unify model metadata, joint names, calibration/config loading, and simulator observations; then reconcile documentation and remove dead code.

Hardware-specific questions remain: which firmware/reference convention produces the reported ~12-rad samples, whether other CAN writers are running concurrently, and which motor-side communication watchdog is configured. None was assumed verified in this audit.
