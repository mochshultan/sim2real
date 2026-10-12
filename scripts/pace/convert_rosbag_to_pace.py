#!/usr/bin/env python3
"""Convert a passive Jaguar ROS 2 bag into PACE's aligned trajectory format.

This utility is offline and never publishes ROS messages or commands motors.
Run it after sourcing the ROS 2 workspace on a machine with rosbag2 Python,
sensor message definitions, NumPy, and PyTorch installed.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import torch


PACE_JOINT_ORDER = (
    "Fr_roll_joint",
    "Fr_hip_pitch_joint",
    "Fr_knee_joint",
    "Fl_roll_joint",
    "Fl_hip_pitch_joint",
    "Fl_knee_joint",
    "Br_roll_joint",
    "Br_hip_pitch_joint",
    "Br_knee_joint",
    "Bl_roll_joint",
    "Bl_hip_pitch_joint",
    "Bl_knee_joint",
)


def canonical_joint_name(name: str) -> str:
    """Normalize supported hardware and Isaac joint names to PACE names."""
    canonical = name.strip().casefold()
    canonical = canonical.replace("_collar_joint", "_roll_joint")
    canonical = canonical.replace("_hip_joint", "_hip_pitch_joint")
    return canonical


def reorder_joint_values(names: list[str], values: list[float], topic: str) -> np.ndarray:
    """Validate and reorder a named JointState vector into PACE joint order."""
    if len(names) != len(values):
        raise ValueError(f"{topic}: {len(names)} joint names but {len(values)} values")
    normalized_names = [canonical_joint_name(name) for name in names]
    if len(set(normalized_names)) != len(normalized_names):
        raise ValueError(f"{topic}: duplicate joint names after normalization: {names}")
    index_by_name = {name: index for index, name in enumerate(normalized_names)}
    expected = [canonical_joint_name(name) for name in PACE_JOINT_ORDER]
    missing = [name for name in expected if name not in index_by_name]
    if missing:
        raise ValueError(f"{topic}: missing Jaguar joints {missing}; received {names}")
    ordered = np.asarray([values[index_by_name[name]] for name in expected], dtype=np.float64)
    if not np.isfinite(ordered).all():
        raise ValueError(f"{topic}: joint values contain NaN or infinity")
    return ordered


def _load_bag_messages(
    bag_path: Path,
    states_topic: str,
    commands_topic: str,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Read named joint states and commands from an SQLite ROS 2 bag."""
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as error:
        raise RuntimeError(
            "ROS 2 bag Python packages are missing. Source the robot ROS 2 setup and retry."
        ) from error

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    absent = [topic for topic in (states_topic, commands_topic) if topic not in topic_types]
    if absent:
        raise ValueError(f"Bag is missing required topic(s): {absent}")
    message_types = {
        topic: get_message(topic_types[topic]) for topic in (states_topic, commands_topic)
    }

    state_times: list[float] = []
    state_positions: list[np.ndarray] = []
    command_times: list[float] = []
    command_positions: list[np.ndarray] = []
    while reader.has_next():
        topic, serialized, timestamp_ns = reader.read_next()
        if topic not in message_types:
            continue
        message = deserialize_message(serialized, message_types[topic])
        timestamp = timestamp_ns * 1.0e-9
        if topic == states_topic:
            position = reorder_joint_values(list(message.name), list(message.position), topic)
            state_times.append(timestamp)
            state_positions.append(position)
        else:
            position = reorder_joint_values(list(message.name), list(message.position), topic)
            command_times.append(timestamp)
            command_positions.append(position)

    if len(state_times) < 2 or len(command_times) < 2:
        raise ValueError(
            f"Insufficient messages: {states_topic}={len(state_times)}, "
            f"{commands_topic}={len(command_times)}"
        )
    return (
        np.asarray(state_times),
        np.stack(state_positions),
        np.asarray(command_times),
        np.stack(command_positions),
    )


def _sort_unique_samples(times: np.ndarray, values: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    order = np.argsort(times, kind="stable")
    sorted_times = times[order]
    sorted_values = values[order]
    unique_times, reverse_indices = np.unique(sorted_times[::-1], return_index=True)
    keep_indices = len(sorted_times) - 1 - reverse_indices
    keep_indices.sort()
    return sorted_times[keep_indices], sorted_values[keep_indices]


def align_trajectory(
    state_times: np.ndarray,
    state_positions: np.ndarray,
    command_times: np.ndarray,
    command_positions: np.ndarray,
    sample_rate_hz: float,
    max_state_gap_s: float,
    max_command_age_s: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Resample measured position and zero-order-held targets onto a fixed grid."""
    state_times, state_positions = _sort_unique_samples(state_times, state_positions)
    command_times, command_positions = _sort_unique_samples(command_times, command_positions)
    dt = 1.0 / sample_rate_hz
    start = max(state_times[0], command_times[0])
    end = min(state_times[-1], command_times[-1] + max_command_age_s)
    if end <= start:
        raise ValueError("State and command timestamps do not overlap")
    grid = np.arange(start, end + 0.25 * dt, dt, dtype=np.float64)

    state_right = np.searchsorted(state_times, grid, side="left")
    state_left = np.clip(state_right - 1, 0, len(state_times) - 1)
    state_right = np.clip(state_right, 0, len(state_times) - 1)
    state_gap = state_times[state_right] - state_times[state_left]
    command_index = np.searchsorted(command_times, grid, side="right") - 1
    command_index_clipped = np.clip(command_index, 0, len(command_times) - 1)
    command_age = grid - command_times[command_index_clipped]
    valid = (
        (grid >= state_times[0])
        & (grid <= state_times[-1])
        & (state_gap <= max_state_gap_s)
        & (command_index >= 0)
        & (command_age <= max_command_age_s)
    )
    grid = grid[valid]
    state_left = state_left[valid]
    state_right = state_right[valid]
    command_index = command_index[valid]
    state_gap = state_gap[valid]
    if len(grid) < 2:
        raise ValueError("Fewer than two aligned samples remain after timestamp-gap checks")

    left_times = state_times[state_left]
    alpha = np.divide(
        grid - left_times,
        state_gap,
        out=np.zeros_like(grid),
        where=state_gap > 0.0,
    )
    measured = (
        state_positions[state_left] * (1.0 - alpha[:, None])
        + state_positions[state_right] * alpha[:, None]
    )
    targets = command_positions[command_index]
    if not np.isfinite(measured).all() or not np.isfinite(targets).all():
        raise ValueError("Aligned trajectory contains NaN or infinite values")
    return (grid - grid[0]).astype(np.float32), measured.astype(np.float32), targets.astype(np.float32)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="ROS 2 bag directory recorded with SQLite storage")
    parser.add_argument("--output", type=Path, required=True, help="Destination PACE .pt file")
    parser.add_argument("--states-topic", default="/joint_states")
    parser.add_argument("--commands-topic", default="/joint_commands")
    parser.add_argument("--sample-rate", type=float, default=200.0, help="Resampled frequency [Hz]")
    parser.add_argument("--max-state-gap", type=float, default=0.02, help="Maximum interpolation gap [s]")
    parser.add_argument("--max-command-age", type=float, default=0.10, help="Maximum held-command age [s]")
    args = parser.parse_args()

    if not args.bag.exists():
        parser.error(f"Bag path does not exist: {args.bag}")
    if args.sample_rate <= 0.0 or args.max_state_gap <= 0.0 or args.max_command_age <= 0.0:
        parser.error("sample rate and timestamp-gap limits must be positive")

    state_times, state_positions, command_times, command_positions = _load_bag_messages(
        args.bag.resolve(), args.states_topic, args.commands_topic
    )
    time_data, dof_pos, desired_dof_pos = align_trajectory(
        state_times,
        state_positions,
        command_times,
        command_positions,
        args.sample_rate,
        args.max_state_gap,
        args.max_command_age,
    )

    output_path = args.output.expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    torch.save(
        {
            "time": torch.from_numpy(time_data),
            "dof_pos": torch.from_numpy(dof_pos),
            "des_dof_pos": torch.from_numpy(desired_dof_pos),
            "metadata": {
                "joint_order": list(PACE_JOINT_ORDER),
                "sample_rate_hz": args.sample_rate,
                "states_topic": args.states_topic,
                "commands_topic": args.commands_topic,
                "source_bag": str(args.bag.resolve()),
            },
        },
        output_path,
    )
    print(f"Saved {len(time_data)} aligned samples ({time_data[-1]:.2f} s) to {output_path}")
    print(f"PACE tensors: time={time_data.shape}, dof_pos={dof_pos.shape}, des_dof_pos={desired_dof_pos.shape}")


if __name__ == "__main__":
    main()
