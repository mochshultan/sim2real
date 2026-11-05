#!/usr/bin/env bash
# Source this helper from build/bringup scripts before loading the workspace.
if [[ -n "${ROS_DISTRO:-}" && "$ROS_DISTRO" != jazzy ]]; then
    echo "Error: ROS $ROS_DISTRO is already sourced. Open a clean Jazzy shell first." >&2
    return 1
fi
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    echo "Error: ROS 2 Jazzy is missing. Install ROS 2 Jazzy for Ubuntu 24.04 first." >&2
    return 1
fi
JAGUAR_WORKSPACE_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
# Leave the old auto-activated environment when reloading an existing terminal.
# Jazzy uses system Python and the user's default site-packages.
if [[ "${VIRTUAL_ENV:-}" == "$JAGUAR_WORKSPACE_DIR/.venv-jazzy" ]]; then
    if ! declare -F deactivate >/dev/null; then
        echo "Error: cannot deactivate the old Jazzy venv. Open a new terminal." >&2
        unset JAGUAR_WORKSPACE_DIR
        return 1
    fi
    deactivate
fi
unset JAGUAR_WORKSPACE_DIR
source /opt/ros/jazzy/setup.bash || return $?
