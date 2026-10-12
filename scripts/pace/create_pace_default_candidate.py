#!/usr/bin/env python3
"""Write the nominal Jaguar PACE actuator candidate for held-out comparison."""

from __future__ import annotations

import argparse
from pathlib import Path

import torch


FIT_JOINTS = ["Fr_roll_joint", "Fr_hip_pitch_joint", "Fr_knee_joint"]


def make_default_candidate() -> dict:
    """Return nominal uncalibrated dynamics from the current Jaguar PACE task."""
    return {
        "iteration": -1,
        "objective_score_rad2": 0.0,
        "joint_order": FIT_JOINTS,
        "armature": torch.full((3,), 0.01, dtype=torch.float32),
        "viscous_friction": torch.zeros(3, dtype=torch.float32),
        "static_dynamic_friction": torch.zeros(3, dtype=torch.float32),
        "encoder_bias_rad": torch.zeros(3, dtype=torch.float32),
        "delay_steps": 0.0,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True, help="Output .pt candidate path")
    args = parser.parse_args()
    output_path = args.output.expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    torch.save(make_default_candidate(), output_path)
    print(f"Saved nominal uncalibrated Jaguar candidate to {output_path}")


if __name__ == "__main__":
    main()
