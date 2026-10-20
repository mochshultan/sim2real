#!/usr/bin/env bash
# Source this helper from build/bringup scripts before loading the workspace.
if [[ -n "${ROS_DISTRO:-}" && "$ROS_DISTRO" != jazzy ]]; then
    echo "Error: ROS $ROS_DISTRO is already sourced. Open a clean Jazzy shell first." >&2
    return 1
fi
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    echo "Error: ROS 2 Jazzy is missing. Run: bash scripts/setup_jazzy.sh" >&2
    return 1
fi
source /opt/ros/jazzy/setup.bash || return $?
JAGUAR_WORKSPACE_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
if [[ -f "$JAGUAR_WORKSPACE_DIR/.venv-jazzy/bin/activate" ]]; then
    source "$JAGUAR_WORKSPACE_DIR/.venv-jazzy/bin/activate"
fi
unset JAGUAR_WORKSPACE_DIR
