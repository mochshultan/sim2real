#!/usr/bin/env bash
# Build and source script for jaguar_control package

(return 0 2>/dev/null) && IS_SOURCED=1 || IS_SOURCED=0

if [ "$IS_SOURCED" -eq 0 ]; then
    set -e
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$WORKSPACE_DIR"

echo "====================================================="
echo " Building NXP Jaguar Control (ROS 2 Humble)          "
echo "====================================================="

echo "[1/3] Sourcing ROS 2 Humble..."
source /opt/ros/humble/setup.bash

echo "[2/3] Running colcon build..."
colcon build --packages-select jaguar_control --symlink-install

echo "[3/3] Sourcing install/setup.bash..."
source install/setup.bash

echo "====================================================="
echo " Build completed successfully."
echo "====================================================="

if [ "$IS_SOURCED" -eq 0 ]; then
    echo "Note: Script executed in a subshell."
    echo "To load environment into current shell, run:"
    echo "  source scripts/build.sh"
fi
