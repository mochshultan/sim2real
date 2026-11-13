#!/usr/bin/env bash
# Launch the current policy, or the previous nominal policy with --old.
set -e

WORKSPACE_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
USE_OLD=false
LAUNCH_ARGS=()
for arg in "$@"; do
    case "$arg" in
        --old) USE_OLD=true ;;
        -h|--help)
            echo "Usage: sim2real [--old] [ROS launch arguments...]"
            echo "  --old  Use nominal policy 29996 instead of the current default."
            exit 0
            ;;
        *) LAUNCH_ARGS+=("$arg") ;;
    esac
done

source "$WORKSPACE_DIR/scripts/source_ros.sh"
source "$WORKSPACE_DIR/install/setup.bash"

POLICY_ARGS=()
if "$USE_OLD"; then
    POLICY_ARGS=(
        "policy_path:=$WORKSPACE_DIR/models/policy_adaboot_ideal_l2c2_kp45kd45_clear8cm_29996_20261022.pt"
        "policy_reference:=nominal"
        "history_initialization:=zeros"
    )
fi
exec ros2 launch jaguar_control sim2real.launch.py "${POLICY_ARGS[@]}" "${LAUNCH_ARGS[@]}"
