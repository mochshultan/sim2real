"""Regression tests for the PACE chirp and timestamp-alignment tools."""

from __future__ import annotations

import csv
import json
from pathlib import Path
import sys

import numpy as np


PACE_SCRIPTS = Path(__file__).resolve().parents[1] / "scripts" / "pace"
sys.path.insert(0, str(PACE_SCRIPTS))

import convert_rosbag_to_pace as converter  # noqa: E402
import analyze_pace_replay as replay_analysis  # noqa: E402
import create_pace_default_candidate as baseline_candidate  # noqa: E402
import run_jaguar_chirp as chirp  # noqa: E402
import run_jaguar_pace_calibration as calibration_workflow  # noqa: E402
import torch  # noqa: E402


def test_chirp_profiles_stay_inside_existing_motion_guards() -> None:
    baseline = 0.0
    sample_times = np.arange(0.0, chirp.DURATION_S, 1.0 / chirp.COMMAND_RATE_HZ)
    for profile in chirp.PROFILES.values():
        targets = np.asarray(
            [chirp.chirp_target(t, 0, baseline, chirp.AMPLITUDE_RAD, profile) for t in sample_times]
        )
        velocities = np.diff(targets) * chirp.COMMAND_RATE_HZ
        assert np.isfinite(targets).all()
        assert np.max(np.abs(targets - baseline)) <= chirp.MAX_AMPLITUDE_RAD
        assert np.max(np.abs(velocities)) <= chirp.CHIRP_COMMAND_VELOCITY_LIMIT_RAD_S


def test_converter_crops_alignment_to_held_out_chirp_window() -> None:
    source_times = np.arange(0.0, 2.001, 0.005)
    positions = np.repeat(source_times[:, None], 12, axis=1)
    sample_time, measured, targets = converter.align_trajectory(
        source_times + 1_800_000_000.0,
        positions,
        source_times + 1_800_000_000.0,
        positions + 0.1,
        sample_rate_hz=200.0,
        max_state_gap_s=0.02,
        max_command_age_s=0.10,
        window_start_unix_s=1_800_000_000.5,
        window_end_unix_s=1_800_000_001.5,
    )
    assert sample_time[0] == 0.0
    assert sample_time[-1] <= 1.0
    assert measured.shape == targets.shape == (len(sample_time), 12)
    assert np.allclose(targets - measured, 0.1)


def test_chirp_csv_window_uses_active_rows_and_metadata(tmp_path: Path) -> None:
    csv_path = tmp_path / "jaguar_chirp_val_01_commands.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=["phase", "unix_time_ns"])
        writer.writeheader()
        writer.writerow({"phase": "stand_transition", "unix_time_ns": 100})
        writer.writerow({"phase": "fr_chirp", "unix_time_ns": 200})
        writer.writerow({"phase": "fr_chirp", "unix_time_ns": 300})
        writer.writerow({"phase": "post_chirp", "unix_time_ns": 400})
    csv_path.with_name("jaguar_chirp_val_01_metadata.json").write_text(
        json.dumps({"profile": "validation"}), encoding="utf-8"
    )

    start, end, metadata = converter.read_chirp_window(csv_path)

    assert np.isclose(start, 200e-9)
    assert np.isclose(end, 300e-9)
    assert metadata["profile"] == "validation"


def test_replay_metrics_include_position_and_velocity_errors() -> None:
    time_data = torch.arange(5, dtype=torch.float64) * 0.01
    measured = torch.zeros((5, 3), dtype=torch.float64)
    simulated = torch.stack((time_data, 2.0 * time_data, 3.0 * time_data), dim=1)
    replay = {
        "candidate_path": "candidate.pt",
        "time": time_data,
        "joint_order": ["roll", "hip", "knee"],
        "measured_dof_pos": measured,
        "simulated_dof_pos": simulated,
        "residual_dof_pos": simulated,
    }

    metrics = replay_analysis.compute_replay_metrics(replay)

    assert metrics["position_rmse_rad"]["roll"] > 0.0
    assert metrics["velocity_rmse_rad_s"] == {"roll": 1.0, "hip": 2.0, "knee": 3.0}


def test_default_candidate_matches_current_pace_nominal_parameters() -> None:
    candidate = baseline_candidate.make_default_candidate()
    assert candidate["joint_order"] == ["Fr_roll_joint", "Fr_hip_pitch_joint", "Fr_knee_joint"]
    assert torch.all(candidate["armature"] == 0.01)
    assert torch.all(candidate["viscous_friction"] == 0.0)
    assert torch.all(candidate["static_dynamic_friction"] == 0.0)
    assert torch.all(candidate["encoder_bias_rad"] == 0.0)
    assert candidate["delay_steps"] == 0.0


def test_calibration_comparison_requires_both_held_out_metrics_to_improve() -> None:
    nominal = {"mean_position_rmse_rad": 0.02, "mean_velocity_rmse_rad_s": 0.4}
    better = {"mean_position_rmse_rad": 0.01, "mean_velocity_rmse_rad_s": 0.2}
    worse_velocity = {"mean_position_rmse_rad": 0.01, "mean_velocity_rmse_rad_s": 0.5}

    assert calibration_workflow.compare_metrics(nominal, better)["improved_both_metrics"]
    assert not calibration_workflow.compare_metrics(nominal, worse_velocity)["improved_both_metrics"]


def test_candidate_bounds_flag_values_at_search_boundary(tmp_path: Path) -> None:
    candidate_path = tmp_path / "candidate.pt"
    torch.save(
        {
            "joint_order": calibration_workflow.JOINT_ORDER,
            "armature": torch.tensor([0.01, 0.02, 0.03]),
            "viscous_friction": torch.tensor([0.0, 1.0, 2.0]),
            "static_dynamic_friction": torch.tensor([0.1, 0.2, 0.3]),
            "encoder_bias_rad": torch.tensor([0.0, 0.01, -0.01]),
            "delay_steps": 0.0,
        },
        candidate_path,
    )

    bounds = calibration_workflow._candidate_bounds(candidate_path)

    assert bounds["viscous_friction"]["near_bound"] == ["Fr_roll_joint"]
    assert bounds["delay_steps"]["near_bound"] == ["shared"]
