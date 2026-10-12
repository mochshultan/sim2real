#!/usr/bin/env python3
"""Summarize position and velocity errors from a PACE candidate replay."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import torch


def compute_replay_metrics(replay: dict) -> dict:
    """Compute per-joint position and velocity RMSE from a PACE replay."""
    time_data = torch.as_tensor(replay["time"], dtype=torch.float64).flatten()
    residual = torch.as_tensor(replay["residual_dof_pos"], dtype=torch.float64)
    measured = torch.as_tensor(replay["measured_dof_pos"], dtype=torch.float64)
    simulated = torch.as_tensor(replay["simulated_dof_pos"], dtype=torch.float64)
    joint_order = list(replay["joint_order"])

    if time_data.ndim != 1 or time_data.numel() < 3:
        raise ValueError("Replay must contain at least three timestamped samples")
    if residual.ndim != 2 or residual.shape != measured.shape or measured.shape != simulated.shape:
        raise ValueError("Replay position arrays must have matching [time, joint] shapes")
    if residual.shape != (time_data.numel(), len(joint_order)):
        raise ValueError("Replay dimensions do not match its timestamps and joint_order")
    if not all(torch.isfinite(value).all() for value in (time_data, residual, measured, simulated)):
        raise ValueError("Replay arrays contain NaN or infinity")
    time_steps = torch.diff(time_data)
    if torch.any(time_steps <= 0.0):
        raise ValueError("Replay timestamps must be strictly increasing")

    measured_velocity = torch.diff(measured, dim=0) / time_steps[:, None]
    simulated_velocity = torch.diff(simulated, dim=0) / time_steps[:, None]
    velocity_error = simulated_velocity - measured_velocity
    per_joint_position_rmse = torch.mean(residual.square(), dim=0).sqrt()
    per_joint_velocity_rmse = torch.mean(velocity_error.square(), dim=0).sqrt()

    return {
        "candidate_path": replay.get("candidate_path"),
        "sample_count": int(time_data.numel()),
        "duration_s": float((time_data[-1] - time_data[0]).item()),
        "joint_order": joint_order,
        "position_rmse_rad": {
            name: float(value) for name, value in zip(joint_order, per_joint_position_rmse.tolist())
        },
        "velocity_rmse_rad_s": {
            name: float(value) for name, value in zip(joint_order, per_joint_velocity_rmse.tolist())
        },
        "mean_position_rmse_rad": float(per_joint_position_rmse.mean().item()),
        "mean_velocity_rmse_rad_s": float(per_joint_velocity_rmse.mean().item()),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("replay", type=Path, help="PACE *_replay.pt output file")
    parser.add_argument("--output", type=Path, default=None, help="Metrics JSON destination")
    args = parser.parse_args()
    if not args.replay.is_file():
        parser.error(f"Replay file does not exist: {args.replay}")

    replay = torch.load(args.replay, map_location="cpu", weights_only=False)
    metrics = compute_replay_metrics(replay)
    output_path = args.output or args.replay.with_name(f"{args.replay.stem}_metrics.json")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(metrics, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(metrics, indent=2))
    print(f"Saved metrics to {output_path}")


if __name__ == "__main__":
    main()
