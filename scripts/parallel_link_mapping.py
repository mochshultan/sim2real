"""Affine coordinate mapping for the Jaguar hip-knee parallel linkage."""

from __future__ import annotations


def motor_to_virtual_delta(
    hip_motor_delta: float,
    knee_motor_delta: float,
    knee_ratio: float = 1.0,
) -> tuple[float, float]:
    """Convert motor deltas to virtual serial-joint deltas.

    Args:
        hip_motor_delta: Hip motor displacement from the captured baseline [rad].
        knee_motor_delta: Knee motor displacement from the captured baseline [rad].
        knee_ratio: Virtual knee displacement per knee motor displacement.

    Returns:
        Hip and relative knee-extension displacements [rad].
    """
    hip_delta = float(hip_motor_delta)
    knee_delta = knee_ratio * float(knee_motor_delta)
    return hip_delta, knee_delta


def virtual_to_motor_delta(
    hip_delta: float,
    knee_delta: float,
    knee_ratio: float = 1.0,
) -> tuple[float, float]:
    """Convert virtual serial-joint deltas to motor deltas.

    Args:
        hip_delta: Virtual hip displacement from the captured baseline [rad].
        knee_delta: Virtual knee displacement from the captured baseline [rad].
        knee_ratio: Virtual knee displacement per knee motor displacement.

    Returns:
        Hip and knee motor displacements from their baselines [rad].
    """
    hip_motor_delta = float(hip_delta)
    knee_motor_delta = float(knee_delta) / knee_ratio
    return hip_motor_delta, knee_motor_delta
