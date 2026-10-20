#!/usr/bin/env bash
# Install the Ubuntu 24.04 runtime/build dependencies; never starts robot nodes.
set -euo pipefail

source /etc/os-release
WORKSPACE_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
if [[ "${ID}:${VERSION_ID}" != "ubuntu:24.04" ]]; then
    echo "This installer requires Ubuntu 24.04." >&2
    exit 1
fi

sudo -v
sudo dpkg --audit

# Preserve the existing repository key and change only the ROS repository suite.
# Do not enable Jammy repositories for ROS on Noble.
ROS_SOURCE=/etc/apt/sources.list.d/ros2.sources
if [[ ! -f "$ROS_SOURCE" ]]; then
    echo "Missing $ROS_SOURCE; configure the official ROS 2 apt repository first." >&2
    exit 1
fi
BACKUP_DIR="$HOME/.local/state/sim2real/jazzy-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$BACKUP_DIR"
sudo cp -a "$ROS_SOURCE" "$BACKUP_DIR/ros2.sources"
SOURCE_TMP=$(mktemp)
trap 'rm -f "$SOURCE_TMP"' EXIT
sed -E 's/^Suites:.*/Suites: noble/; s/^Enabled: no$/Enabled: yes/' "$ROS_SOURCE" > "$SOURCE_TMP"
sudo install -m 644 "$SOURCE_TMP" "$ROS_SOURCE"

sudo apt update
sudo apt install \
    ros-jazzy-desktop ros-jazzy-twist-mux ros-jazzy-rqt-plot ros-jazzy-joy \
    ros-dev-tools build-essential cmake libyaml-cpp-dev \
    python3-numpy python3-scipy python3-yaml python3-can \
    python3-pytest python3-venv

# CPU inference only; retain access to apt-installed ROS and scientific modules.
/usr/bin/python3 -m venv --system-site-packages "$WORKSPACE_DIR/.venv-jazzy"
touch "$WORKSPACE_DIR/.venv-jazzy/COLCON_IGNORE"
"$WORKSPACE_DIR/.venv-jazzy/bin/python" -m pip install \
    'torch==2.9.1' --index-url https://download.pytorch.org/whl/cpu

echo "Jazzy installed. Repository backup: $BACKUP_DIR"
echo "The workspace still needs a clean rebuild in a Jazzy-only shell."
