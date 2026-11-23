#!/usr/bin/env bash
# Current-position deployment: 48D actor by default, or 64D DreamWaQ with --64d.
set -e

WORKSPACE_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
MODE=48d
LAUNCH_ARGS=()
for arg in "$@"; do
    case "$arg" in
        --old) MODE=old ;;
        --48d) MODE=48d ;;
        --64d) MODE=64d ;;
        -h|--help)
            echo "Usage: sim2real [--48d|--64d|--old] [ROS launch arguments...]"
            echo "  --48d  Current-position actions with the 48D actor (default)."
            echo "  --64d  Current-position actions with the 64D DreamWaQ actor."
            echo "  --old  Use nominal policy 29996 instead of the current default."
            exit 0
            ;;
        *) LAUNCH_ARGS+=("$arg") ;;
    esac
done

source "$WORKSPACE_DIR/scripts/source_ros.sh"
source "$WORKSPACE_DIR/install/setup.bash"
source "$WORKSPACE_DIR/scripts/runtime_python.sh" sim2real

POLICY_ARGS=()
LAUNCH_FILE="sim2real_${MODE}.launch.py"
if [[ "$MODE" == old ]]; then
    LAUNCH_FILE="sim2real.launch.py"
    POLICY_ARGS=(
        "policy_path:=$WORKSPACE_DIR/models/policy_adaboot_ideal_l2c2_kp45kd45_clear8cm_29996_20261022.pt"
        "policy_reference:=nominal"
        "history_initialization:=zeros"
        "config_file:=$WORKSPACE_DIR/config/sim2real.yaml"
        "controller_executable:=nxp_jaguar_controller.py"
        "controller_name:=nxp_jaguar_controller"
        "actor_input_dim:=0"
    )
fi
exec ros2 launch jaguar_control "$LAUNCH_FILE" "${POLICY_ARGS[@]}" "${LAUNCH_ARGS[@]}"
