#!/bin/bash
# ==============================================================================
# 🐾 NXP Jaguar: IMU Bringup Script (ROS 2 Serial IMU Driver)
# ==============================================================================

set -e

# ANSI Color Codes
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}=====================================================${NC}"
echo -e "${GREEN} 🐾 NXP JAGUAR: SERIAL IMU BRINGUP                   ${NC}"
echo -e "${GREEN}=====================================================${NC}"

# 1. Check Serial Port Device
IMU_PORT="/dev/ttyUSB0"
if [ ! -e "$IMU_PORT" ]; then
    # Check if there is another ttyUSB port available
    ALT_PORT=$(ls /dev/ttyUSB* 2>/dev/null | head -n 1 || true)
    if [ -n "$ALT_PORT" ]; then
        echo -e "${YELLOW}[WARN] Port $IMU_PORT tidak ditemukan, menggunakan: $ALT_PORT${NC}"
        IMU_PORT="$ALT_PORT"
    else
        echo -e "${RED}[ERROR] Port IMU ($IMU_PORT) tidak terdeteksi!${NC}"
        echo -e "Pastikan kabel USB IMU sudah tercolok dengan kencang."
        exit 1
    fi
fi

# Ensure user has access permission to port
sudo chmod 666 "$IMU_PORT" 2>/dev/null || true
echo -e "✅ Port Serial IMU Terdeteksi: ${GREEN}$IMU_PORT${NC}"

# 2. Source ROS 2 Base Environment
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo -e "✅ ROS 2 Humble Environment Loaded."
elif [ -f "/opt/ros/foxy/setup.bash" ]; then
    source /opt/ros/foxy/setup.bash
    echo -e "✅ ROS 2 Foxy Environment Loaded."
else
    echo -e "${RED}[ERROR] ROS 2 setup.bash tidak ditemukan di /opt/ros!${NC}"
    exit 1
fi

# 3. Source Workspace Overlay with serial_imu package
if [ -f "/home/erc/sim2real/install/setup.bash" ]; then
    source /home/erc/sim2real/install/setup.bash
elif [ -f "/home/erc/nxp_jaguar_2/nxp_jaguar/install/setup.bash" ]; then
    source /home/erc/nxp_jaguar_2/nxp_jaguar/install/setup.bash
elif [ -f "/home/erc/nxp_jaguar/install/setup.bash" ]; then
    source /home/erc/nxp_jaguar/install/setup.bash
else
    echo -e "${RED}[ERROR] Workspace serial_imu tidak ditemukan!${NC}"
    exit 1
fi

echo -e "✅ serial_imu Package Loaded."
echo -e "📡 Memulai publishing topik: ${GREEN}/Imu_data${NC} (sensor_msgs/msg/Imu)..."
echo -e "${YELLOW}Tekan Ctrl+C untuk menghentikan driver IMU.${NC}"
echo "-----------------------------------------------------"

# 4. Launch serial_imu talker node
exec ros2 run serial_imu talker
