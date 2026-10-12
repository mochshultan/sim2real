#!/usr/bin/env bash

# Record Jaguar joint commands and encoder feedback for offline PACE fitting.
# This script is passive: it subscribes to ROS topics and never publishes commands.

set -Eeuo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
label="${1:-jaguar_joint_data}"
if [[ ! "${label}" =~ ^[A-Za-z0-9._-]+$ ]]; then
    echo "Label may contain only letters, numbers, dot, underscore, and hyphen." >&2
    exit 2
fi

if ! command -v ros2 >/dev/null 2>&1; then
    echo "ros2 was not found. Source the robot's ROS 2 and workspace setup first." >&2
    exit 127
fi

timestamp="$(date +%Y%m%d_%H%M%S)"
output_dir="${repo_dir}/data/pace/${label}_${timestamp}"
mkdir -p "$(dirname -- "${output_dir}")"

echo "Recording passive PACE data to: ${output_dir}"
echo "Topics: /joint_states /joint_commands /jaguar/motor_diagnostics /jaguar/hardware_status"
echo "No motor commands are sent. Press Ctrl+C to stop recording."
exec ros2 bag record --storage sqlite3 --output "${output_dir}" \
    /joint_states \
    /joint_commands \
    /jaguar/motor_diagnostics \
    /jaguar/hardware_status
