#!/usr/bin/env python3
"""Run Jaguar PACE fitting and independent held-out replay on the PC.

This utility is offline: it converts two separately recorded robot runs, fits
only the fitting run, compares nominal and fitted candidates on the held-out
run, and writes an auditable JSON/Markdown report. It never commands hardware
or installs fitted parameters into the robot controller.
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
import time
from pathlib import Path

import torch


TASK_ID = "Isaac-Pace-NXP-Jaguar-v0"
JOINT_ORDER = ["Fr_roll_joint", "Fr_hip_pitch_joint", "Fr_knee_joint"]
PACE_BOUNDS = {
    "armature": [1.0e-5, 1.0],
    "viscous_friction": [0.0, 7.0],
    "static_dynamic_friction": [0.0, 0.5],
    "encoder_bias_rad": [-0.1, 0.1],
    "delay_steps": [0.0, 10.0],
}


def _run(command: list[str], cwd: Path) -> None:
    """Run a command and stop immediately if it fails."""
    print("+", " ".join(command), flush=True)
    subprocess.run(command, cwd=cwd, check=True)


def _candidate_bounds(candidate_path: Path, margin_fraction: float = 0.02) -> dict:
    """Report fitted parameters near the configured Jaguar task bounds."""
    candidate = torch.load(candidate_path, map_location="cpu", weights_only=False)
    if candidate.get("joint_order") != JOINT_ORDER:
        raise ValueError(
            f"Expected FR-only candidate joint order {JOINT_ORDER}; got {candidate.get('joint_order')}"
        )
    results = {}
    for name, bounds in PACE_BOUNDS.items():
        if name == "delay_steps":
            values = [float(candidate[name])]
        else:
            values = torch.as_tensor(candidate[name], dtype=torch.float64).flatten().tolist()
        low, high = bounds
        tolerance = (high - low) * margin_fraction
        near = [
            index
            for index, value in enumerate(values)
            if value < low - 1.0e-8 or value > high + 1.0e-8
        ]
        if near:
            raise ValueError(f"Candidate {name} values fall outside PACE bounds: {values}")
        at_bound = [
            ("shared" if name == "delay_steps" else JOINT_ORDER[index])
            for index, value in enumerate(values)
            if value - low <= tolerance or high - value <= tolerance
        ]
        results[name] = {"values": values, "near_bound": at_bound, "bounds": bounds}
    return results


def compare_metrics(nominal: dict, fitted: dict) -> dict:
    """Compare held-out candidate errors without imposing arbitrary RMSE cutoffs."""
    position_gain = 1.0 - fitted["mean_position_rmse_rad"] / max(
        nominal["mean_position_rmse_rad"], 1.0e-12
    )
    velocity_gain = 1.0 - fitted["mean_velocity_rmse_rad_s"] / max(
        nominal["mean_velocity_rmse_rad_s"], 1.0e-12
    )
    return {
        "position_rmse_relative_improvement": position_gain,
        "velocity_rmse_relative_improvement": velocity_gain,
        "improved_both_metrics": position_gain > 0.0 and velocity_gain > 0.0,
    }


def _load_dataset_summary(dataset_path: Path, expected_profile: str) -> dict:
    """Check an aligned PACE dataset and return its measurement summary."""
    dataset = torch.load(dataset_path, map_location="cpu", weights_only=False)
    time_data = torch.as_tensor(dataset["time"], dtype=torch.float64).flatten()
    measured = torch.as_tensor(dataset["dof_pos"], dtype=torch.float64)
    desired = torch.as_tensor(dataset["des_dof_pos"], dtype=torch.float64)
    metadata = dataset.get("metadata", {})
    chirp_metadata = metadata.get("chirp_metadata", {})
    if chirp_metadata.get("profile") != expected_profile:
        raise ValueError(
            f"{dataset_path} must be a {expected_profile!r} chirp dataset; "
            f"metadata says {chirp_metadata.get('profile')!r}"
        )
    if list(metadata.get("joint_order", [])) != [
        "Fr_roll_joint", "Fr_hip_pitch_joint", "Fr_knee_joint",
        "Fl_roll_joint", "Fl_hip_pitch_joint", "Fl_knee_joint",
        "Br_roll_joint", "Br_hip_pitch_joint", "Br_knee_joint",
        "Bl_roll_joint", "Bl_hip_pitch_joint", "Bl_knee_joint",
    ]:
        raise ValueError(f"{dataset_path} has unexpected PACE joint ordering")
    if measured.shape != desired.shape or measured.shape != (time_data.numel(), 12):
        raise ValueError(f"{dataset_path} must contain aligned [time, 12] joint arrays")
    if time_data.numel() < 1000 or not torch.isfinite(time_data).all():
        raise ValueError(f"{dataset_path} has too few or invalid timestamp samples")
    if not torch.isfinite(measured).all() or not torch.isfinite(desired).all():
        raise ValueError(f"{dataset_path} has non-finite joint data")
    if torch.any(torch.diff(time_data) <= 0.0):
        raise ValueError(f"{dataset_path} timestamps must be strictly increasing")
    return {
        "path": str(dataset_path.resolve()),
        "profile": expected_profile,
        "sample_count": int(time_data.numel()),
        "duration_s": float(time_data[-1] - time_data[0]),
        "sample_rate_hz": metadata.get("sample_rate_hz"),
        "source_bag": metadata.get("source_bag"),
        "source_command_csv": metadata.get("source_command_csv"),
    }


def _new_fit_run(log_root: Path, before: set[Path], started_at: float) -> Path:
    """Find exactly one new PACE fit directory created by this invocation."""
    after = {path for path in log_root.iterdir() if path.is_dir()}
    new_runs = [path for path in after - before if path.stat().st_mtime >= started_at - 2.0]
    if len(new_runs) != 1:
        raise RuntimeError(f"Expected one new PACE fit run, found: {[str(p) for p in new_runs]}")
    candidates = [
        path for path in new_runs[0].glob("best_candidate*.pt")
        if not path.name.endswith("_replay.pt")
    ]
    if len(candidates) != 1:
        raise RuntimeError(f"Expected one fitted candidate in {new_runs[0]}, found {candidates}")
    return candidates[0]


def _write_report(output_dir: Path, report: dict) -> None:
    """Write structured results and a concise human-readable summary."""
    (output_dir / "calibration_report.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    nominal = report["held_out_comparison"]["nominal"]
    fitted = report["held_out_comparison"]["fitted"]
    gains = report["held_out_comparison"]["relative_improvement"]
    lines = [
        "# NXP Jaguar PACE calibration report",
        "",
        f"Status: **{report['status']}**",
        "",
        f"Fit joints: {', '.join(report['fit_joints'])}",
        f"Fit data: {report['datasets']['fit']['sample_count']} samples, "
        f"{report['datasets']['fit']['duration_s']:.2f} s",
        f"Held-out data: {report['datasets']['validation']['sample_count']} samples, "
        f"{report['datasets']['validation']['duration_s']:.2f} s",
        "",
        "| Candidate | Position RMSE (rad) | Velocity RMSE (rad/s) |",
        "|---|---:|---:|",
        f"| Nominal | {nominal['mean_position_rmse_rad']:.6g} | "
        f"{nominal['mean_velocity_rmse_rad_s']:.6g} |",
        f"| Fitted | {fitted['mean_position_rmse_rad']:.6g} | "
        f"{fitted['mean_velocity_rmse_rad_s']:.6g} |",
        "",
        f"Held-out relative improvement: position {gains['position_rmse_relative_improvement']:.1%}, "
        f"velocity {gains['velocity_rmse_relative_improvement']:.1%}.",
        "",
        "This report identifies only the three excited front-right joints. It does not "
        "validate all 12 actuators, ground-contact behavior, or a locomotion policy. "
        "The candidate is not installed into robot controller configuration.",
        "",
        f"Candidate: `{report['candidate_path']}`",
        f"Nominal replay: `{report['replays']['nominal']}`",
        f"Fitted replay: `{report['replays']['fitted']}`",
    ]
    (output_dir / "calibration_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fit-bag", type=Path, required=True)
    parser.add_argument("--fit-csv", type=Path, required=True)
    parser.add_argument("--validation-bag", type=Path, required=True)
    parser.add_argument("--validation-csv", type=Path, required=True)
    parser.add_argument("--isaaclab-root", type=Path, default=Path("/home/shultan/IsaacLab"))
    parser.add_argument("--pace-root", type=Path, default=Path("/home/shultan/pace-sim2real"))
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--num-envs", type=int, default=32)
    parser.add_argument("--max-iterations", type=int, default=200)
    args = parser.parse_args()

    isaaclab_root = args.isaaclab_root.expanduser().resolve()
    pace_root = args.pace_root.expanduser().resolve()
    fit_bag = args.fit_bag.expanduser().resolve()
    fit_csv = args.fit_csv.expanduser().resolve()
    validation_bag = args.validation_bag.expanduser().resolve()
    validation_csv = args.validation_csv.expanduser().resolve()
    output_dir = args.output_dir.expanduser().resolve()
    for path in (fit_bag, fit_csv, validation_bag, validation_csv):
        if not path.exists():
            parser.error(f"Input does not exist: {path}")
    if not (isaaclab_root / "isaaclab.sh").is_file():
        parser.error(f"Isaac Lab launcher missing under {isaaclab_root}")
    fit_script = pace_root / "scripts" / "pace" / "fit.py"
    if not fit_script.is_file():
        parser.error(f"PACE fit.py not found: {fit_script}")
    if args.num_envs < 1 or args.max_iterations < 1:
        parser.error("--num-envs and --max-iterations must be positive")
    output_dir.mkdir(parents=True, exist_ok=False)
    datasets_dir = output_dir / "datasets"
    datasets_dir.mkdir()
    fit_data = datasets_dir / "fit.pt"
    validation_data = datasets_dir / "validation.pt"

    launcher = str(isaaclab_root / "isaaclab.sh")
    converter = Path(__file__).resolve().with_name("convert_rosbag_to_pace.py")
    for bag, csv_path, output in (
        (fit_bag, fit_csv, fit_data),
        (validation_bag, validation_csv, validation_data),
    ):
        _run(
            [
                launcher, "-p", str(converter), str(bag),
                "--command-csv", str(csv_path), "--output", str(output),
            ],
            isaaclab_root,
        )
    fit_summary = _load_dataset_summary(fit_data, "fit")
    validation_summary = _load_dataset_summary(validation_data, "validation")

    nominal_path = output_dir / "nominal_candidate.pt"
    nominal_script = Path(__file__).resolve().with_name("create_pace_default_candidate.py")
    _run([launcher, "-p", str(nominal_script), "--output", str(nominal_path)], isaaclab_root)

    log_root = pace_root / "logs" / "pace" / "nxp_jaguar"
    log_root.mkdir(parents=True, exist_ok=True)
    previous_runs = {path for path in log_root.iterdir() if path.is_dir()}
    started_at = time.time()
    _run(
        [
            launcher, "-p", str(fit_script), "--headless", "--viz", "none",
            "--task", TASK_ID, "--num_envs", str(args.num_envs),
            "--max_iterations", str(args.max_iterations), "--data", str(fit_data),
        ],
        isaaclab_root,
    )
    fitted_source = _new_fit_run(log_root, previous_runs, started_at)
    fitted_path = output_dir / "fitted_candidate.pt"
    shutil.copy2(fitted_source, fitted_path)

    replays = {}
    for name, candidate in (("nominal", nominal_path), ("fitted", fitted_path)):
        _run(
            [
                launcher, "-p", str(fit_script), "--headless", "--viz", "none",
                "--task", TASK_ID, "--num_envs", "1", "--data", str(validation_data),
                "--evaluate_candidate", str(candidate),
            ],
            isaaclab_root,
        )
        replay_path = candidate.with_name(f"{candidate.stem}_replay.pt")
        metrics_script = Path(__file__).resolve().with_name("analyze_pace_replay.py")
        metrics_path = output_dir / f"{name}_held_out_metrics.json"
        _run(
            [launcher, "-p", str(metrics_script), str(replay_path), "--output", str(metrics_path)],
            isaaclab_root,
        )
        replays[name] = {"path": str(replay_path), "metrics": json.loads(metrics_path.read_text())}

    comparison = compare_metrics(replays["nominal"]["metrics"], replays["fitted"]["metrics"])
    bounds = _candidate_bounds(fitted_path)
    has_bound_warning = any(value["near_bound"] for value in bounds.values())
    report = {
        "status": "PROVISIONAL",
        "status_reasons": [
            "one fitting/held-out recording pair is not repeatability evidence",
            "only FR collar, hip, and knee were excited and fitted",
        ],
        "fit_joints": JOINT_ORDER,
        "datasets": {"fit": fit_summary, "validation": validation_summary},
        "candidate_path": str(fitted_path),
        "candidate_parameters": bounds,
        "candidate_near_parameter_bounds": has_bound_warning,
        "held_out_comparison": {
            "nominal": replays["nominal"]["metrics"],
            "fitted": replays["fitted"]["metrics"],
            "relative_improvement": comparison,
        },
        "replays": {name: value["path"] for name, value in replays.items()},
        "fit_run_directory": str(fitted_source.parent),
        "fit_iterations_requested": args.max_iterations,
    }
    if not comparison["improved_both_metrics"]:
        report["status_reasons"].append(
            "fitted candidate did not improve both held-out position and velocity RMSE"
        )
    if has_bound_warning:
        report["status_reasons"].append(
            "one or more fitted parameters lie within 2% of a configured search bound"
        )
    _write_report(output_dir, report)
    print(f"Calibration report: {output_dir / 'calibration_report.md'}")
    print(f"Status: {report['status']}; held-out gain: {comparison}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
