#!/usr/bin/env bash
# ==============================================================================
# Build and source script for jaguar_control ROS 2 workspace
# ==============================================================================

(return 0 2>/dev/null) && IS_SOURCED=1 || IS_SOURCED=0

if [ "$IS_SOURCED" -eq 0 ]; then
    set -e
fi

# Locate workspace root directory
SCRIPT_SOURCE="${BASH_SOURCE[0]}"
REAL_SCRIPT_PATH="$(readlink -f "$SCRIPT_SOURCE" 2>/dev/null || realpath "$SCRIPT_SOURCE" 2>/dev/null || echo "$SCRIPT_SOURCE")"
SCRIPT_DIR="$(cd "$(dirname "$REAL_SCRIPT_PATH")" && pwd)"

if [ -f "$SCRIPT_DIR/CMakeLists.txt" ]; then
    WORKSPACE_DIR="$SCRIPT_DIR"
elif [ -f "$SCRIPT_DIR/../CMakeLists.txt" ]; then
    WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
else
    WORKSPACE_DIR="/home/erc/sim2real"
fi

cd "$WORKSPACE_DIR"

echo "====================================================="
echo " Building NXP Jaguar Control (ROS 2 Humble)          "
echo "====================================================="

echo "[1/3] Sourcing ROS 2..."
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
elif [ -f "/opt/ros/foxy/setup.bash" ]; then
    source /opt/ros/foxy/setup.bash
else
    echo "Warning: No ROS 2 installation found in /opt/ros."
fi

echo "[2/3] Running colcon build..."
CMAKE_ARGS=""
if [ -x "/usr/bin/python3" ]; then
    CMAKE_ARGS="--cmake-args -DPython3_EXECUTABLE=/usr/bin/python3"
fi

if [ -d "$WORKSPACE_DIR/serial_imu" ]; then
    colcon build --base-paths . serial_imu --packages-select jaguar_control serial_imu --symlink-install $CMAKE_ARGS || { status=$?; return "$status" 2>/dev/null || exit "$status"; }
else
    colcon build --packages-select jaguar_control --symlink-install $CMAKE_ARGS || { status=$?; return "$status" 2>/dev/null || exit "$status"; }
fi

echo "[3/3] Sourcing install/setup.bash..."
if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    source "$WORKSPACE_DIR/install/setup.bash"
fi

echo "====================================================="
echo " Build completed successfully."
echo "====================================================="

if [ "$IS_SOURCED" -eq 0 ]; then
    echo "Note: Script executed in a subshell."
    echo "To load environment into current shell, run:"
    echo "  source scripts/build.sh"
fi
