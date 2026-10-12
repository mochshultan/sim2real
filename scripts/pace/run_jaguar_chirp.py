#!/usr/bin/env python3
"""Run a guarded, low-amplitude Jaguar joint chirp for PACE data collection.

The script is opt-in (``--arm`` plus a five-second auto-start countdown),
requires the C++ CAN driver's torque-limit setting to be 3 Nm, and refuses to run if another
node is publishing motor commands. Firmware readback must be verified by the
operator. It only supports a suspended robot.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys
import time


JOINTS = (
    "BL_collar_joint", "BL_hip_joint", "BL_knee_joint",
    "BR_collar_joint", "BR_hip_joint", "BR_knee_joint",
    "FL_collar_joint", "FL_hip_joint", "FL_knee_joint",
    "FR_collar_joint", "FR_hip_joint", "FR_knee_joint",
)
ACTIVE_JOINT_INDICES = (9, 10, 11)  # FR: collar, hip, knee in the hardware ordering above.
ALL_JOINT_INDICES = tuple(range(12))
NOMINAL_STAND_POSITION = (
    0.0, -1.49, 1.20,  # BL
    0.0, -1.49, 1.20,  # BR
    0.0, -1.51, 1.22,  # FL
    0.0, -1.51, 1.22,  # FR
)

# Conservative intersection of the current controller, URDF, and driver limits.
JOINT_LIMITS = tuple(
    limit
    for _leg in range(4)
    for limit in ((-0.40, 0.40), (-2.50, 0.00), (-0.10, 2.50))
)

AMPLITUDE_RAD = 0.03
MAX_AMPLITUDE_RAD = 0.03
START_FREQUENCY_HZ = 0.10
END_FREQUENCY_HZ = 0.80
DURATION_S = 20.0
STAND_TRANSITION_DURATION_S = 18.0
COMMAND_RATE_HZ = 50.0
DRIVER_TORQUE_LIMIT_NM = 3.0
STOP_TORQUE_NM = 2.5
STOP_VELOCITY_RAD_S = 0.30
CHIRP_COMMAND_VELOCITY_LIMIT_RAD_S = 0.15
START_SETTLED_VELOCITY_RAD_S = 0.10
STOP_TRACKING_ERROR_RAD = 0.05
STOP_TEMPERATURE_C = 65.0
FEEDBACK_TIMEOUT_S = 0.10
POSITION_MARGIN_RAD = 0.10
DRIVER_POSITION_TOLERANCE_RAD = 0.20
CHIRP_KP = 40.0
CHIRP_KD = 2.5


@dataclass(frozen=True)
class ChirpProfile:
    """A bounded excitation profile for a recorded system-identification run."""

    name: str
    start_frequency_hz: float
    end_frequency_hz: float


PROFILES = {
    "fit": ChirpProfile("fit", START_FREQUENCY_HZ, END_FREQUENCY_HZ),
    "validation": ChirpProfile("validation", END_FREQUENCY_HZ, START_FREQUENCY_HZ),
}


def chirp_envelope(t: float, duration: float, ramp: float = 2.0) -> float:
    """Smoothly fade chirp amplitude in and out."""
    if t <= 0.0 or t >= duration:
        return 0.0
    if t < ramp:
        return math.sin(0.5 * math.pi * t / ramp) ** 2
    if t > duration - ramp:
        return math.sin(0.5 * math.pi * (duration - t) / ramp) ** 2
    return 1.0


def chirp_target(
    t: float,
    joint_index: int,
    baseline: float,
    amplitude: float,
    profile: ChirpProfile,
) -> float:
    """Linear frequency sweep with a distinct fixed phase per joint."""
    phase_offset = 2.0 * math.pi * joint_index / len(ACTIVE_JOINT_INDICES)
    phase = 2.0 * math.pi * (
        profile.start_frequency_hz * t
        + (profile.end_frequency_hz - profile.start_frequency_hz)
        * t
        * t
        / (2.0 * DURATION_S)
    ) + phase_offset
    return baseline + amplitude * chirp_envelope(t, DURATION_S) * math.sin(phase)


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--arm", action="store_true",
        help="Enable the physical chirp after ROS preflight and a five-second countdown.",
    )
    parser.add_argument(
        "--amplitude", type=float, default=AMPLITUDE_RAD,
        help=f"Peak excursion in radians (default {AMPLITUDE_RAD}; hard cap {MAX_AMPLITUDE_RAD}).",
    )
    parser.add_argument(
        "--profile", choices=tuple(PROFILES), default="fit",
        help="Use an ascending fit chirp or descending held-out validation chirp.",
    )
    parser.add_argument(
        "--label", default=None,
        help="Short dataset label included in the CSV and metadata filenames.",
    )
    args = parser.parse_args()
    if not math.isfinite(args.amplitude) or not 0.0 < args.amplitude <= MAX_AMPLITUDE_RAD:
        parser.error(f"--amplitude must be in (0, {MAX_AMPLITUDE_RAD}] rad")
    if args.label is not None and not all(c.isalnum() or c in "._-" for c in args.label):
        parser.error("--label may contain only letters, numbers, dot, underscore, and hyphen")
    return args


def _print_plan(amplitude: float, profile: ChirpProfile, label: str | None = None) -> None:
    print("NXP Jaguar PACE chirp plan")
    print("  approach: all 12 joints move to the configured nominal standing pose")
    print("  chirp: FR collar, hip, and knee; other 9 joints hold the standing pose")
    print(f"  stand transition: {STAND_TRANSITION_DURATION_S:.1f} s")
    print(f"  excursion: +/-{amplitude:.3f} rad")
    print(
        f"  profile: {profile.name}; frequency: "
        f"{profile.start_frequency_hz:.2f} -> {profile.end_frequency_hz:.2f} Hz"
    )
    print(f"  duration: {DURATION_S:.1f} s; command rate: {COMMAND_RATE_HZ:.0f} Hz")
    if label:
        print(f"  dataset label: {label}")
    print(f"  required firmware torque limit: {DRIVER_TORQUE_LIMIT_NM:.1f} Nm")
    print(f"  active-joint MIT gains: Kp={CHIRP_KP:.1f}, Kd={CHIRP_KD:.1f}")
    print(f"  chirp target-speed guard: {CHIRP_COMMAND_VELOCITY_LIMIT_RAD_S:.2f} rad/s")
    print(f"  stop thresholds: {STOP_TORQUE_NM:.1f} Nm, {STOP_VELOCITY_RAD_S:.2f} rad/s, "
          f"{STOP_TRACKING_ERROR_RAD:.2f} rad tracking error, {STOP_TEMPERATURE_C:.0f} C")


class JaguarChirp:
    """ROS 2 command publisher with feedback watchdogs."""

    def __init__(self, amplitude: float):
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool, Float32, Float32MultiArray, String

        self.rclpy = rclpy
        self.JointState = JointState
        self.Bool = Bool
        self.amplitude = amplitude
        self.node = Node("jaguar_pace_chirp")
        self.positions: list[float] | None = None
        self.velocities: list[float] | None = None
        self.temperatures: list[float] | None = None
        self.feedback_time = 0.0
        self.temperature_time = 0.0
        self.hardware_status = ""
        self.status_time = 0.0
        self.torque_limit: float | None = None
        self.torque_limit_time = 0.0
        self.command_pub = self.node.create_publisher(JointState, "/joint_commands", 10)
        self.active_motion_started = False
        self.estop_pub = self.node.create_publisher(Bool, "/jaguar/emergency_stop", 10)

        self.node.create_subscription(JointState, "/joint_states", self._joint_cb, 10)
        self.node.create_subscription(Float32MultiArray, "/jaguar/motor_diagnostics", self._temp_cb, 10)
        self.node.create_subscription(String, "/jaguar/hardware_status", self._status_cb, 10)
        self.node.create_subscription(Float32, "/jaguar/control_torque_limit_nm", self._limit_cb, 10)

    def _joint_cb(self, msg) -> None:
        if len(msg.name) != 12 or len(msg.position) != 12 or len(msg.velocity) != 12 or len(msg.effort) != 12:
            return
        by_name = {name: i for i, name in enumerate(msg.name)}
        if set(by_name) != set(JOINTS):
            return
        order = [by_name[name] for name in JOINTS]
        values = [msg.position[i] for i in order]
        speeds = [msg.velocity[i] for i in order]
        torques = [msg.effort[i] for i in order]
        if not all(math.isfinite(v) for v in values + speeds + torques):
            return
        self.positions = values
        self.velocities = speeds
        self.torques = torques
        self.feedback_time = time.monotonic()

    def _temp_cb(self, msg) -> None:
        if len(msg.data) != 12 or not all(math.isfinite(v) for v in msg.data):
            return
        self.temperatures = list(msg.data)
        self.temperature_time = time.monotonic()

    def _status_cb(self, msg) -> None:
        self.hardware_status = msg.data
        self.status_time = time.monotonic()

    def _limit_cb(self, msg) -> None:
        if math.isfinite(msg.data):
            self.torque_limit = float(msg.data)
            self.torque_limit_time = time.monotonic()

    def spin(self, timeout: float = 0.02) -> None:
        self.rclpy.spin_once(self.node, timeout_sec=timeout)

    def wait_for_preflight(self) -> None:
        deadline = time.monotonic() + 60.0
        next_keepalive = 0.0
        print("No competing command publisher detected; waiting for C++ CAN driver.")
        print("Passive zero-gain heartbeat starts now. Start the driver in its terminal.")
        while time.monotonic() < deadline and self.rclpy.ok():
            self.spin(0.05)
            now = time.monotonic()
            self.ensure_no_competitors()
            if now >= next_keepalive:
                # Zero gains keep the driver's command watchdog alive without
                # applying motor torque while waiting for fresh feedback.
                self.publish_command(self.positions or [0.0] * 12, [0.0] * 12, gain_scale=0.0)
                next_keepalive = now + 1.0 / COMMAND_RATE_HZ
            if self.hardware_status in ("EMERGENCY_STOPPED", "RESETTING_FAULT"):
                raise RuntimeError(f"Driver reports {self.hardware_status} during preflight.")
            if (self.positions is not None and self.temperatures is not None and
                    self.hardware_status and self.torque_limit is not None and
                    now - self.feedback_time < FEEDBACK_TIMEOUT_S and
                    now - self.temperature_time < 2.5 and now - self.status_time < 2.5 and
                    now - self.torque_limit_time < 2.5):
                return
        raise RuntimeError("Feedback, diagnostics, driver status, or torque-limit report is missing/stale.")

    def validate_preflight(self) -> list[float]:
        self.wait_for_preflight()
        if self.hardware_status != "PASSIVE_ZERO_TORQUE":
            raise RuntimeError(f"Hardware must start passive; reported {self.hardware_status!r}.")
        if abs(self.torque_limit - DRIVER_TORQUE_LIMIT_NM) > 0.05:
            raise RuntimeError(
                f"Driver configured torque limit is {self.torque_limit:.2f} Nm; restart it with "
                f"control_torque_limit_nm:={DRIVER_TORQUE_LIMIT_NM:.1f}."
            )
        if max(abs(tau) for tau in self.torques) >= STOP_TORQUE_NM:
            raise RuntimeError("Measured startup torque is already at/above the chirp stop threshold.")
        if max(abs(self.velocities[i]) for i in ALL_JOINT_INDICES) >= START_SETTLED_VELOCITY_RAD_S:
            moving_joint = max(ALL_JOINT_INDICES, key=lambda i: abs(self.velocities[i]))
            raise RuntimeError(
                f"Joint {JOINTS[moving_joint]} is not settled "
                f"({self.velocities[moving_joint]:+.4f} rad/s; require "
                f"< {START_SETTLED_VELOCITY_RAD_S:.2f} rad/s)."
            )
        if max(self.temperatures) >= STOP_TEMPERATURE_C:
            raise RuntimeError("A motor is at/above the chirp temperature stop threshold.")

        baseline = list(self.positions)
        for i, (position, limits) in enumerate(zip(baseline, JOINT_LIMITS)):
            lower, upper = limits
            excursion = max(lower - position, position - upper, 0.0)
            if excursion > DRIVER_POSITION_TOLERANCE_RAD:
                raise RuntimeError(
                    f"{JOINTS[i]}={position:+.3f} rad is {excursion:.3f} rad outside "
                    "the configured joint range (driver tolerance "
                    f"{DRIVER_POSITION_TOLERANCE_RAD:.2f} rad); the CAN driver's "
                    "safe-park interlock would trigger. Resolve encoder zero/limits first."
                )
        for i in ACTIVE_JOINT_INDICES:
            position = baseline[i]
            limits = JOINT_LIMITS[i]
            lower, upper = limits
            if not lower + POSITION_MARGIN_RAD < position < upper - POSITION_MARGIN_RAD:
                raise RuntimeError(
                    f"{JOINTS[i]} baseline {position:+.3f} rad is too close to/outside "
                    f"the safe interval [{lower + POSITION_MARGIN_RAD:+.2f}, "
                    f"{upper - POSITION_MARGIN_RAD:+.2f}] rad."
                )
            if position - self.amplitude < lower + POSITION_MARGIN_RAD or \
                    position + self.amplitude > upper - POSITION_MARGIN_RAD:
                raise RuntimeError(f"Requested chirp excursion does not fit safely for {JOINTS[i]}.")

        return list(self.positions)

    def check_no_competitors(self) -> None:
        # Do not publish during graph discovery: even a passive message can
        # race a controller that is already commanding the robot.
        graph_deadline = time.monotonic() + 1.0
        while time.monotonic() < graph_deadline:
            self.spin(0.05)
            self.ensure_no_competitors()

    def ensure_no_competitors(self) -> None:
        for topic in ("/joint_commands", "/mit_controller/commands"):
            publishers = self.node.get_publishers_info_by_topic(topic)
            publishers = [p for p in publishers if p.node_name != self.node.get_name()]
            if publishers:
                names = ", ".join(info.node_name for info in publishers)
                raise RuntimeError(f"Competing publisher(s) on {topic}: {names}. Stop them first.")

    def countdown_while_keepalive(self, seconds: float = 5.0) -> bool:
        print(
            f"Auto-start in {seconds:.0f} seconds. Confirm rosbag is recording and "
            "the 3 Nm firmware limit was verified. Press Ctrl+C to cancel."
        )
        deadline = time.monotonic() + seconds
        last_remaining = math.ceil(seconds) + 1
        next_keepalive = 0.0
        while self.rclpy.ok() and time.monotonic() < deadline:
            self.spin(0.001)
            self.check_feedback(self.positions or [0.0] * 12)
            now = time.monotonic()
            if now >= next_keepalive:
                self.publish_command(self.positions or [0.0] * 12, [0.0] * 12, gain_scale=0.0)
                next_keepalive = now + 1.0 / COMMAND_RATE_HZ
            remaining = math.ceil(deadline - now)
            if remaining < last_remaining:
                print(f"  {remaining}...")
                last_remaining = remaining
            time.sleep(0.005)
        return self.rclpy.ok()

    def publish_command(
        self,
        position: list[float],
        velocity: list[float],
        gain_scale: float = 1.0,
        hold_all_joints: bool = False,
    ) -> None:
        msg = self.JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = list(JOINTS)
        msg.position = position
        msg.velocity = velocity
        kp = [0.0] * 12
        kd = [0.0] * 12
        gain_indices = ALL_JOINT_INDICES if hold_all_joints else ACTIVE_JOINT_INDICES
        for index in gain_indices:
            kp[index] = CHIRP_KP
            kd[index] = CHIRP_KD
        msg.effort = [gain_scale * v for v in kp] + [gain_scale * v for v in kd] + [0.0] * 12
        self.command_pub.publish(msg)

    def check_feedback(
        self,
        target: list[float],
        guarded_indices: tuple[int, ...] = ACTIVE_JOINT_INDICES,
        position_guarded_indices: tuple[int, ...] | None = None,
    ) -> None:
        now = time.monotonic()
        if (self.positions is None or self.velocities is None or self.torques is None or
                now - self.feedback_time > FEEDBACK_TIMEOUT_S):
            raise RuntimeError("Joint feedback timed out (> 0.1 s).")
        if self.temperatures is None or now - self.temperature_time > 2.5:
            raise RuntimeError("Motor-temperature diagnostics timed out.")
        if max(abs(tau) for tau in self.torques) >= STOP_TORQUE_NM:
            raise RuntimeError(f"Measured torque reached {STOP_TORQUE_NM:.1f} Nm stop threshold.")
        if max(abs(v) for v in self.velocities) >= STOP_VELOCITY_RAD_S:
            raise RuntimeError(f"Measured velocity reached {STOP_VELOCITY_RAD_S:.2f} rad/s stop threshold.")
        if max(self.temperatures) >= STOP_TEMPERATURE_C:
            raise RuntimeError(f"Motor temperature reached {STOP_TEMPERATURE_C:.0f} C stop threshold.")
        if max(abs(self.positions[i] - target[i]) for i in guarded_indices) >= STOP_TRACKING_ERROR_RAD:
            raise RuntimeError(
                f"Tracking error reached {STOP_TRACKING_ERROR_RAD:.2f} rad stop threshold "
                f"on {', '.join(JOINTS[i] for i in guarded_indices)}."
            )
        if position_guarded_indices is None:
            position_guarded_indices = guarded_indices
        for i in position_guarded_indices:
            position = self.positions[i]
            limits = JOINT_LIMITS[i]
            lower, upper = limits
            if position <= lower + POSITION_MARGIN_RAD or position >= upper - POSITION_MARGIN_RAD:
                raise RuntimeError(f"{JOINTS[i]} reached the protected joint-limit margin.")
        if self.hardware_status in ("EMERGENCY_STOPPED", "RESETTING_FAULT"):
            raise RuntimeError(f"Driver reports {self.hardware_status}.")
        self.ensure_no_competitors()

    def emergency_stop(self) -> None:
        msg = self.Bool()
        msg.data = True
        self.estop_pub.publish(msg)

    def close(self) -> None:
        self.node.destroy_node()


def run_physical(amplitude: float, profile: ChirpProfile, label: str | None) -> int:
    try:
        import rclpy
    except ImportError as error:
        raise RuntimeError("ROS 2 Python libraries are unavailable; source ROS 2 first.") from error

    rclpy.init()
    chirp = JaguarChirp(amplitude)
    try:
        chirp.check_no_competitors()
        baseline = chirp.validate_preflight()
        _print_plan(amplitude, profile, label)
        print("\nMeasured baseline (hardware order BL, BR, FL, FR):")
        for name, value in zip(JOINTS, baseline):
            print(f"  {name}: {value:+.4f} rad")
        print("\nPreflight passed. Robot must remain fully suspended; controller publishers are absent.")
        if not chirp.countdown_while_keepalive():
            print("Cancelled before active motion.")
            return 2

        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and chirp.command_pub.get_subscription_count() < 1:
            chirp.spin(0.05)
        if chirp.command_pub.get_subscription_count() < 1:
            raise RuntimeError("No hardware-driver subscriber found on /joint_commands.")

        output_dir = Path(__file__).resolve().parents[2] / "data" / "pace"
        output_dir.mkdir(parents=True, exist_ok=True)
        stamp = f"{time.strftime('%Y%m%d_%H%M%S')}_{time.time_ns() % 1_000_000_000:09d}"
        run_label = label or profile.name
        stem = f"jaguar_chirp_{run_label}_{stamp}"
        csv_path = output_dir / f"{stem}_commands.csv"
        metadata_path = output_dir / f"{stem}_metadata.json"
        fields = [
            "phase", "time_s", "unix_time_ns", "monotonic_time_ns",
            *[f"target_{name}_rad" for name in JOINTS],
        ]
        period = 1.0 / COMMAND_RATE_HZ
        prior_targets = baseline
        print(f"\nRunning stand transition, then FR chirp. Command trajectory: {csv_path}")
        with csv_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()

            transition_started = time.monotonic()
            while True:
                loop_started = time.monotonic()
                elapsed = loop_started - transition_started
                if elapsed >= STAND_TRANSITION_DURATION_S:
                    break
                progress = min(1.0, max(0.0, elapsed / STAND_TRANSITION_DURATION_S))
                blend = 0.5 * (1.0 - math.cos(math.pi * progress))
                targets = [
                    start + blend * (goal - start)
                    for start, goal in zip(baseline, NOMINAL_STAND_POSITION)
                ]
                gain_scale = min(1.0, max(0.0, elapsed / 1.5))
                velocities = [(targets[i] - prior_targets[i]) / period for i in range(12)]
                if max(abs(v) for v in velocities) > 0.15:
                    raise RuntimeError("Stand transition target velocity exceeded 0.15 rad/s guard.")
                chirp.publish_command(
                    targets, velocities, gain_scale=gain_scale, hold_all_joints=True
                )
                if gain_scale > 0.0:
                    chirp.active_motion_started = True
                chirp.spin(0.001)
                chirp.check_feedback(
                    targets,
                    guarded_indices=ALL_JOINT_INDICES,
                    position_guarded_indices=(),
                )
                row = {
                    "phase": "stand_transition",
                    "time_s": f"{elapsed:.6f}",
                    "unix_time_ns": time.time_ns(),
                    "monotonic_time_ns": time.monotonic_ns(),
                }
                row.update(
                    {f"target_{name}_rad": f"{value:.8f}" for name, value in zip(JOINTS, targets)}
                )
                writer.writerow(row)
                stream.flush()
                prior_targets = targets
                sleep_time = period - (time.monotonic() - loop_started)
                if sleep_time > 0.0:
                    time.sleep(sleep_time)

            stand_targets = list(NOMINAL_STAND_POSITION)
            for _ in range(5):
                chirp.publish_command(stand_targets, [0.0] * 12, hold_all_joints=True)
                chirp.spin(0.02)
                chirp.check_feedback(stand_targets, guarded_indices=ALL_JOINT_INDICES)
            print("\nNominal standing pose reached and held. Start rosbag now:")
            print(f"  bash scripts/pace/record_jaguar_joint_data.sh pace_{run_label}")
            print("FR chirp will start automatically in 5 seconds; Ctrl+C cancels.")
            settle_deadline = time.monotonic() + 5.0
            while time.monotonic() < settle_deadline:
                chirp.spin(0.005)
                chirp.check_feedback(stand_targets, guarded_indices=ALL_JOINT_INDICES)
                chirp.publish_command(stand_targets, [0.0] * 12, hold_all_joints=True)
                time.sleep(0.01)

            chirp_started = time.monotonic()
            chirp_start_unix_ns = time.time_ns()
            prior_targets = stand_targets
            print(f"\nRunning FR chirp. Command trajectory saved to {csv_path}")
            while True:
                loop_started = time.monotonic()
                elapsed = loop_started - chirp_started
                if elapsed >= DURATION_S:
                    break
                targets = list(stand_targets)
                for phase_index, joint_index in enumerate(ACTIVE_JOINT_INDICES):
                    targets[joint_index] = chirp_target(
                        elapsed, phase_index, stand_targets[joint_index], amplitude, profile
                    )
                velocities = [(targets[i] - prior_targets[i]) / period for i in range(12)]
                if max(abs(v) for v in velocities) > CHIRP_COMMAND_VELOCITY_LIMIT_RAD_S:
                    raise RuntimeError(
                        "Generated chirp target velocity exceeded "
                        f"{CHIRP_COMMAND_VELOCITY_LIMIT_RAD_S:.2f} rad/s guard."
                    )
                chirp.publish_command(targets, velocities, hold_all_joints=True)
                chirp.spin(0.001)
                chirp.check_feedback(targets, guarded_indices=ALL_JOINT_INDICES)
                row = {
                    "phase": "fr_chirp",
                    "time_s": f"{elapsed:.6f}",
                    "unix_time_ns": time.time_ns(),
                    "monotonic_time_ns": time.monotonic_ns(),
                }
                row.update(
                    {f"target_{name}_rad": f"{value:.8f}" for name, value in zip(JOINTS, targets)}
                )
                writer.writerow(row)
                stream.flush()
                prior_targets = targets
                sleep_time = period - (time.monotonic() - loop_started)
                if sleep_time > 0.0:
                    time.sleep(sleep_time)

            chirp_end_unix_ns = time.time_ns()

        # Keep all joints in the standing pose until the driver watchdog stops.
        for _ in range(3):
            chirp.publish_command(list(NOMINAL_STAND_POSITION), [0.0] * 12, hold_all_joints=True)
            chirp.spin(0.02)
        metadata = {
            "schema_version": 1,
            "label": run_label,
            "profile": profile.name,
            "chirp_start_unix_ns": chirp_start_unix_ns,
            "chirp_end_unix_ns": chirp_end_unix_ns,
            "active_joint_names": [JOINTS[index] for index in ACTIVE_JOINT_INDICES],
            "held_joint_names": [
                name for i, name in enumerate(JOINTS) if i not in ACTIVE_JOINT_INDICES
            ],
            "amplitude_rad": amplitude,
            "start_frequency_hz": profile.start_frequency_hz,
            "end_frequency_hz": profile.end_frequency_hz,
            "chirp_duration_s": DURATION_S,
            "command_rate_hz": COMMAND_RATE_HZ,
            "kp": CHIRP_KP,
            "kd": CHIRP_KD,
            "driver_torque_limit_nm": DRIVER_TORQUE_LIMIT_NM,
            "joint_order": list(JOINTS),
            "command_csv": csv_path.name,
        }
        metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
        print(
            "Chirp complete. Robot remains at nominal stand until driver watchdog stop. "
            f"CSV: {csv_path}; metadata: {metadata_path}"
        )
        chirp.active_motion_started = False
        return 0
    except KeyboardInterrupt:
        print("\nInterrupted; commanding passive zero torque.")
        if chirp.command_pub is not None:
            positions = chirp.positions or [0.0] * 12
            for _ in range(3):
                chirp.publish_command(positions, [0.0] * 12, gain_scale=0.0)
                chirp.spin(0.02)
        return 130
    except Exception as error:
        print(f"[SAFETY STOP] {error}", file=sys.stderr)
        if chirp.command_pub is not None and chirp.active_motion_started:
            chirp.emergency_stop()
            chirp.spin(0.05)
        elif chirp.command_pub is not None and chirp.positions is not None:
            for _ in range(3):
                chirp.publish_command(chirp.positions, [0.0] * 12, gain_scale=0.0)
                chirp.spin(0.02)
        return 2
    finally:
        chirp.close()
        rclpy.shutdown()


def main() -> int:
    args = _arguments()
    profile = PROFILES[args.profile]
    _print_plan(args.amplitude, profile, args.label)
    if not args.arm:
        print("\nPreview only. Physical commands require --arm and a five-second auto-start countdown.")
        return 0
    if not sys.stdin.isatty():
        print("[ABORT] Physical chirp requires an interactive terminal.", file=sys.stderr)
        return 2
    return run_physical(args.amplitude, profile, args.label)


if __name__ == "__main__":
    raise SystemExit(main())
