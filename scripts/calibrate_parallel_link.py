#!/usr/bin/env python3
"""Safely test the affine hip-knee coupling on one suspended Jaguar leg.

The operator supports the selected leg while its current pose is captured. The
script then holds that pose with low-gain impedance control. Hip commands leave
the knee motor target fixed; knee commands test the 1:1 remote transmission to
the tibia.
"""

from __future__ import annotations

import argparse
import atexit
import csv
import math
import os
from pathlib import Path
import select
import signal
import sys
import termios
import time
import tty
from typing import TYPE_CHECKING

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import parameters as P
from parallel_link_mapping import motor_to_virtual_delta, virtual_to_motor_delta

if TYPE_CHECKING:
    from robstride_motor_lib import RobStrideMotorController


LEG_INDICES = {
    "BL": (0, 1, 2),
    "BR": (3, 4, 5),
    "FL": (6, 7, 8),
    "FR": (9, 10, 11),
}
JOINT_LIMITS = {
    0: (-0.50, 0.50),
    1: (-2.50, 0.20),
    2: (-0.25, 2.50),
}
COMPETING_CONTROLLERS = (
    "can_hardware_node",
    "nxp_jaguar_controller",
    "robstride_can_node",
    "test_sit_stand.py",
    "calibrate_sit_zero.py",
    "calibrate_stand_pose.py",
    "check_joints.py",
    "robstride_motor_test.py",
    "calibrate_parallel_link.py",
)


class SafetyStop(RuntimeError):
    """Raised when an active calibration safety limit is exceeded."""


def _find_competing_controllers() -> list[str]:
    """Return command lines for other processes that may command the CAN bus."""
    matches: list[str] = []
    own_pid = os.getpid()
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit() or int(entry.name) == own_pid:
            continue
        try:
            command = (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode(errors="replace")
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        if command and any(name in command for name in COMPETING_CONTROLLERS):
            matches.append(f"PID {entry.name}: {command.strip()}")
    return matches


def _move_toward(current: float, target: float, maximum_step: float) -> float:
    """Rate-limit one position command."""
    error = target - current
    if abs(error) <= maximum_step:
        return target
    return current + math.copysign(maximum_step, error)


class ParallelLinkCalibration:
    """Low-gain interactive calibration controller for one suspended leg."""

    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.indices = LEG_INDICES[args.leg]
        self.active_indices = self.indices if args.hold_collar else self.indices[1:]
        self.motors: dict[int, RobStrideMotorController] = {}
        self.position: dict[int, float] = {}
        self.velocity: dict[int, float] = {}
        self.torque: dict[int, float] = {}
        self.temperature: dict[int, float] = {}
        self.baseline: dict[int, float] = {}
        self.command: dict[int, float] = {}
        self.hip_delta_target = 0.0
        self.knee_delta_target = 0.0
        self.running = True
        self.active = False
        self.marker = ""
        self.old_terminal_settings: list[int] | None = None
        self.log_file = None
        self.log_writer = None
        atexit.register(self.disable_all)

    def initialize(self) -> None:
        """Configure only the motors belonging to the selected leg."""
        try:
            from robstride_motor_lib import RobStrideMotorController
        except ModuleNotFoundError as error:
            if error.name == "can":
                raise RuntimeError(
                    "Python package 'can' tidak tersedia. Jalankan di PC robot pada "
                    "environment yang biasa dipakai oleh test_sit_stand.py."
                ) from error
            raise

        for index in self.active_indices:
            motor = RobStrideMotorController(
                bus=P.DEVICE[index],
                motor_id=P.CAN_ID[index],
                motor_type=P.MOTOR_TYPE[index],
                motor_dir=P.MOTOR_DIR[index],
            )
            # Register immediately so a later setup failure still disables it.
            self.motors[index] = motor
            motor.enable_motor()
            motor.set_run_mode("CONTROL_MODE")
            motor.set_angle_offset(P.MOTOR_OFFSET_ANGLE[index])
            motor.set_angle_range(*JOINT_LIMITS[index % 3])

        self._read_passive()
        self._validate_initial_pose()

    def _read_passive(self) -> None:
        for index, motor in self.motors.items():
            result = motor.send_control_command(0.0, 0.0, 0.0, 0.0, 0.0, timeout=15)
            self._store_feedback(index, result)

    def _store_feedback(self, index: int, result: tuple) -> None:
        _, position, velocity, torque, temperature = result
        if None in (position, velocity, torque, temperature):
            raise SafetyStop(f"Tidak ada feedback valid dari {P.JOINT_NAME[index]}.")
        self.position[index] = float(position)
        self.velocity[index] = float(velocity)
        self.torque[index] = float(torque)
        self.temperature[index] = float(temperature)

    def _validate_initial_pose(self) -> None:
        for index, position in self.position.items():
            lower, upper = JOINT_LIMITS[index % 3]
            if not lower <= position <= upper:
                raise SafetyStop(
                    f"{P.JOINT_NAME[index]} berada di luar limit: {position:+.3f} rad "
                    f"(limit {lower:+.2f} sampai {upper:+.2f})."
                )

    def print_capture(self) -> None:
        """Show the passive pose before the operator arms active holding."""
        hip_index, knee_index = self.indices[1:]
        print(f"\nPose terbaca untuk kaki {self.args.leg}:")
        if self.args.hold_collar:
            print(f"  collar = {self.position[self.indices[0]]:+.4f} rad")
        print(f"  hip    = {self.position[hip_index]:+.4f} rad")
        print(f"  knee   = {self.position[knee_index]:+.4f} rad")
        print(f"  knee motor coordinate = {self.position[knee_index]:+.4f} rad")

    def engage_hold(self) -> None:
        """Recapture the supported pose and gradually engage impedance hold."""
        self._read_passive()
        self._validate_initial_pose()
        self.baseline = self.position.copy()
        self.command = self.baseline.copy()

        for motor in self.motors.values():
            motor.enable_motor()

        ramp_steps = max(10, int(self.args.hz * self.args.gain_ramp))
        for step in range(1, ramp_steps + 1):
            gain_fraction = step / ramp_steps
            for index, motor in self.motors.items():
                kp = self.args.collar_kp if index % 3 == 0 else self.args.kp
                kd = self.args.collar_kd if index % 3 == 0 else self.args.kd
                result = motor.send_control_command(
                    self.baseline[index],
                    0.0,
                    kp * gain_fraction,
                    kd * gain_fraction,
                    0.0,
                    timeout=15,
                )
                self._store_feedback(index, result)
                self._check_feedback(index, self.baseline[index])
            time.sleep(1.0 / self.args.hz)
        self.active = True

    def _check_feedback(self, index: int, target: float) -> None:
        motor = self.motors[index]
        if motor.fault_reason is not None:
            raise SafetyStop(f"Fault {P.JOINT_NAME[index]}: {motor.fault_reason}")
        if abs(self.torque[index]) > self.args.max_torque:
            raise SafetyStop(
                f"Overtorque {P.JOINT_NAME[index]}: {self.torque[index]:+.2f} Nm "
                f"> {self.args.max_torque:.2f} Nm."
            )
        if abs(self.velocity[index]) > self.args.max_velocity:
            raise SafetyStop(
                f"Kecepatan {P.JOINT_NAME[index]} terlalu tinggi: "
                f"{self.velocity[index]:+.2f} rad/s."
            )
        if self.temperature[index] > self.args.max_temperature:
            raise SafetyStop(
                f"Temperatur {P.JOINT_NAME[index]} terlalu tinggi: "
                f"{self.temperature[index]:.1f} degC."
            )
        if abs(target - self.position[index]) > self.args.max_tracking_error:
            raise SafetyStop(
                f"Tracking error {P.JOINT_NAME[index]} terlalu besar: "
                f"target={target:+.3f}, aktual={self.position[index]:+.3f} rad."
            )

    def _requested_targets(self) -> dict[int, float]:
        hip_index, knee_index = self.indices[1:]
        hip_motor_delta, knee_motor_delta = virtual_to_motor_delta(
            self.hip_delta_target,
            self.knee_delta_target,
            self.args.knee_ratio,
        )
        targets = self.baseline.copy()
        targets[hip_index] += hip_motor_delta
        targets[knee_index] += knee_motor_delta
        return targets

    def _validate_targets(self, targets: dict[int, float]) -> None:
        for index, target in targets.items():
            lower, upper = JOINT_LIMITS[index % 3]
            if not lower <= target <= upper:
                raise ValueError(f"Target {P.JOINT_NAME[index]} melewati joint limit.")
            if abs(target - self.baseline[index]) > self.args.max_excursion:
                raise ValueError(
                    f"Target {P.JOINT_NAME[index]} melewati excursion "
                    f"{self.args.max_excursion:.2f} rad dari baseline."
                )

    def adjust(self, hip_delta: float = 0.0, knee_delta: float = 0.0) -> None:
        """Adjust virtual targets after validating the complete motor command."""
        old_hip = self.hip_delta_target
        old_knee = self.knee_delta_target
        self.hip_delta_target += hip_delta
        self.knee_delta_target += knee_delta
        try:
            self._validate_targets(self._requested_targets())
        except ValueError as error:
            self.hip_delta_target = old_hip
            self.knee_delta_target = old_knee
            print(f"\n[DITOLAK] {error}")

    def reset_target(self) -> None:
        self.hip_delta_target = 0.0
        self.knee_delta_target = 0.0

    def _open_log(self) -> Path:
        stamp = time.strftime("%Y%m%d_%H%M%S")
        path = Path(self.args.log_dir) / f"{self.args.leg}_{stamp}.csv"
        path.parent.mkdir(parents=True, exist_ok=True)
        self.log_file = path.open("w", newline="", encoding="utf-8")
        fields = [
            "time_s",
            "marker",
            "leg",
            "knee_ratio",
            "hip_target_rad",
            "hip_position_rad",
            "hip_velocity_rad_s",
            "hip_torque_nm",
            "knee_target_rad",
            "knee_position_rad",
            "knee_velocity_rad_s",
            "knee_torque_nm",
            "hip_delta_rad",
            "knee_motor_delta_rad",
            "knee_virtual_delta_rad",
        ]
        self.log_writer = csv.DictWriter(self.log_file, fieldnames=fields)
        self.log_writer.writeheader()
        return path

    def _write_log(self, elapsed: float) -> None:
        hip_index, knee_index = self.indices[1:]
        hip_motor_delta = self.position[hip_index] - self.baseline[hip_index]
        knee_motor_delta = self.position[knee_index] - self.baseline[knee_index]
        _, knee_virtual_delta = motor_to_virtual_delta(
            hip_motor_delta,
            knee_motor_delta,
            self.args.knee_ratio,
        )
        self.log_writer.writerow(
            {
                "time_s": f"{elapsed:.6f}",
                "marker": self.marker,
                "leg": self.args.leg,
                "knee_ratio": self.args.knee_ratio,
                "hip_target_rad": f"{self.command[hip_index]:.7f}",
                "hip_position_rad": f"{self.position[hip_index]:.7f}",
                "hip_velocity_rad_s": f"{self.velocity[hip_index]:.7f}",
                "hip_torque_nm": f"{self.torque[hip_index]:.7f}",
                "knee_target_rad": f"{self.command[knee_index]:.7f}",
                "knee_position_rad": f"{self.position[knee_index]:.7f}",
                "knee_velocity_rad_s": f"{self.velocity[knee_index]:.7f}",
                "knee_torque_nm": f"{self.torque[knee_index]:.7f}",
                "hip_delta_rad": f"{hip_motor_delta:.7f}",
                "knee_motor_delta_rad": f"{knee_motor_delta:.7f}",
                "knee_virtual_delta_rad": f"{knee_virtual_delta:.7f}",
            }
        )
        if self.marker:
            self.log_file.flush()
            self.marker = ""

    def _handle_key(self, key: str) -> None:
        step = self.args.step
        if key.lower() == "a":
            self.adjust(hip_delta=-step)
        elif key.lower() == "d":
            self.adjust(hip_delta=step)
        elif key.lower() == "j":
            self.adjust(knee_delta=-step)
        elif key.lower() == "l":
            self.adjust(knee_delta=step)
        elif key.lower() == "r":
            self.reset_target()
        elif key.lower() == "m":
            self.marker = "manual"
            print("\n[MARK] Sampel ditandai di CSV.")
        elif key.lower() == "q" or key == " ":
            self.running = False
        elif key == "\x03":
            self.running = False

    def _read_key(self) -> str | None:
        readable, _, _ = select.select([sys.stdin], [], [], 0.0)
        return sys.stdin.read(1) if readable else None

    def run(self) -> Path:
        """Run the active calibration loop until the operator stops it."""
        log_path = self._open_log()
        print("\nKontrol:")
        print("  A / D : gerakkan hip saja; target motor knee tetap")
        print("  J / L : gerakkan motor knee melalui parallelogram 1:1")
        print("  R     : kembali perlahan ke baseline")
        print("  M     : tandai sampel di CSV")
        print("  SPACE atau Q: segera passive dan keluar")
        print(f"  step={self.args.step:.3f} rad, excursion maksimum={self.args.max_excursion:.3f} rad")

        self.old_terminal_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        start = time.monotonic()
        next_status = start
        try:
            while self.running:
                loop_start = time.monotonic()
                key = self._read_key()
                if key is not None:
                    self._handle_key(key)
                if not self.running:
                    break

                requested = self._requested_targets()
                maximum_step = self.args.rate / self.args.hz
                for index in self.active_indices:
                    self.command[index] = _move_toward(
                        self.command[index], requested[index], maximum_step
                    )
                    kp = self.args.collar_kp if index % 3 == 0 else self.args.kp
                    kd = self.args.collar_kd if index % 3 == 0 else self.args.kd
                    result = self.motors[index].send_control_command(
                        self.command[index], 0.0, kp, kd, 0.0, timeout=15
                    )
                    self._store_feedback(index, result)
                    self._check_feedback(index, self.command[index])

                elapsed = loop_start - start
                self._write_log(elapsed)
                if loop_start >= next_status:
                    hip_index, knee_index = self.indices[1:]
                    hip_delta = self.position[hip_index] - self.baseline[hip_index]
                    knee_delta = self.position[knee_index] - self.baseline[knee_index]
                    _, knee_virtual_delta = motor_to_virtual_delta(
                        hip_delta, knee_delta, self.args.knee_ratio
                    )
                    print(
                        f"\rhip={self.position[hip_index]:+.3f} "
                        f"knee={self.position[knee_index]:+.3f} | "
                        f"dHip={hip_delta:+.3f} dKnee={knee_delta:+.3f} "
                        f"dKneeVirtual={knee_virtual_delta:+.3f} | "
                        f"tau=({self.torque[hip_index]:+.2f},"
                        f"{self.torque[knee_index]:+.2f}) Nm   ",
                        end="",
                        flush=True,
                    )
                    next_status = loop_start + 0.2

                sleep_time = 1.0 / self.args.hz - (time.monotonic() - loop_start)
                if sleep_time > 0.0:
                    time.sleep(sleep_time)
        finally:
            if self.old_terminal_settings is not None:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_terminal_settings)
                self.old_terminal_settings = None
            self.disable_all()
        return log_path

    def disable_all(self) -> None:
        """Disable every motor opened by this process."""
        if not self.motors:
            return
        self.active = False
        for motor in self.motors.values():
            try:
                motor.disable_motor()
            except Exception:
                pass
        if self.log_file is not None and not self.log_file.closed:
            self.log_file.flush()
            self.log_file.close()
        self.motors.clear()


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Low-gain affine parallel-link test for one suspended Jaguar leg."
    )
    parser.add_argument("--leg", required=True, choices=LEG_INDICES, help="Leg to test.")
    parser.add_argument("--kp", type=float, default=8.0, help="Hip/knee Kp. Default: 8.0")
    parser.add_argument("--kd", type=float, default=0.8, help="Hip/knee Kd. Default: 0.8")
    parser.add_argument("--collar_kp", type=float, default=6.0, help="Collar Kp. Default: 6.0")
    parser.add_argument("--collar_kd", type=float, default=0.6, help="Collar Kd. Default: 0.6")
    parser.add_argument(
        "--no_collar_hold", dest="hold_collar", action="store_false", help="Do not hold collar."
    )
    parser.set_defaults(hold_collar=True)
    parser.add_argument("--step", type=float, default=0.03, help="Key step [rad]. Default: 0.03")
    parser.add_argument(
        "--max_excursion", type=float, default=0.18, help="Maximum motion from baseline [rad]."
    )
    parser.add_argument("--rate", type=float, default=0.12, help="Maximum command rate [rad/s].")
    parser.add_argument("--hz", type=float, default=50.0, help="Control frequency [Hz].")
    parser.add_argument(
        "--knee_ratio", type=float, default=1.0, help="Virtual knee / knee motor motion."
    )
    parser.add_argument("--gain_ramp", type=float, default=1.5, help="Gain ramp duration [s].")
    parser.add_argument("--max_torque", type=float, default=6.0, help="Stop torque [Nm].")
    parser.add_argument(
        "--max_velocity", type=float, default=1.0, help="Stop velocity [rad/s]."
    )
    parser.add_argument(
        "--max_tracking_error", type=float, default=0.25, help="Stop tracking error [rad]."
    )
    parser.add_argument(
        "--max_temperature", type=float, default=65.0, help="Stop motor temperature [degC]."
    )
    parser.add_argument(
        "--log_dir", default="log/parallel_link_calibration", help="CSV output directory."
    )
    args = parser.parse_args()
    positive = (
        "kp",
        "kd",
        "collar_kp",
        "collar_kd",
        "step",
        "max_excursion",
        "rate",
        "hz",
        "gain_ramp",
        "max_torque",
        "max_velocity",
        "max_tracking_error",
        "max_temperature",
    )
    for name in positive:
        if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0.0:
            parser.error(f"--{name} must be finite and positive")
    if not math.isfinite(args.knee_ratio) or args.knee_ratio == 0.0:
        parser.error("--knee_ratio must be finite and non-zero")
    safe_upper_bounds = {
        "kp": 15.0,
        "kd": 2.0,
        "collar_kp": 10.0,
        "collar_kd": 2.0,
        "step": 0.05,
        "max_excursion": 0.25,
        "rate": 0.20,
        "hz": 100.0,
        "max_torque": 8.0,
        "max_velocity": 1.5,
        "max_tracking_error": 0.30,
        "max_temperature": 70.0,
    }
    for name, upper_bound in safe_upper_bounds.items():
        if getattr(args, name) > upper_bound:
            parser.error(f"--{name} exceeds calibration safety limit {upper_bound}")
    return args


def main() -> int:
    args = _parse_args()
    conflicts = _find_competing_controllers()
    if conflicts:
        print("[ABORT] Ada proses lain yang mungkin mengontrol motor:", file=sys.stderr)
        for conflict in conflicts:
            print(f"  {conflict}", file=sys.stderr)
        print("Matikan proses tersebut dahulu; jangan berbagi CAN control.", file=sys.stderr)
        return 2
    if not sys.stdin.isatty():
        print("[ABORT] Script interaktif ini harus dijalankan dari terminal TTY.", file=sys.stderr)
        return 2

    controller = ParallelLinkCalibration(args)

    def stop_handler(_signum, _frame) -> None:
        controller.running = False

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)

    print("=== Jaguar Parallel-Link Calibration ===")
    print("Robot harus tergantung stabil. Jauhkan orang dan benda dari workspace kaki.")
    print(f"Hanya kaki {args.leg} yang akan ditahan; policy/ROS motor node harus mati.")
    try:
        controller.initialize()
        controller.print_capture()
        confirmation = input(
            "\nTopang kaki dengan sling/strap pada pose ini; jauhkan tangan dari linkage. "
            "Ketik HOLD lalu Enter untuk aktifkan low-gain hold: "
        )
        if confirmation.strip() != "HOLD":
            print("Dibatalkan; motor tetap passive.")
            return 1
        controller.engage_hold()
        print("Hold aktif. Longgarkan sling perlahan sambil siap menekan SPACE.")
        log_path = controller.run()
        print(f"\nMotor passive. Log tersimpan: {log_path}")
        return 0
    except (SafetyStop, OSError, RuntimeError) as error:
        print(f"\n[SAFETY STOP] {error}", file=sys.stderr)
        return 3
    finally:
        controller.disable_all()


if __name__ == "__main__":
    raise SystemExit(main())
