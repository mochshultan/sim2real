#!/usr/bin/env bash
# ==============================================================================
# 🐾 NXP Jaguar: 12-Motor Real-Time Telemetry Plotter (rqt_plot)
# Automatically launches rqt_plot configured for all 12 RobStride RS00 motors.
# ==============================================================================

set -e

# Color definitions
GREEN="\033[0;32m"
CYAN="\033[0;36m"
YELLOW="\033[1;33m"
RED="\033[0;31m"
NC="\033[0m"

echo -e "${CYAN}=====================================================${NC}"
echo -e "${CYAN} 🐾 NXP JAGUAR: 12-MOTOR RQT PLOTTER                ${NC}"
echo -e "${CYAN}=====================================================${NC}"

# 1. Source ROS 2 Environment
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
elif [ -f "/opt/ros/foxy/setup.bash" ]; then
    source /opt/ros/foxy/setup.bash
else
    echo -e "${RED}[ERROR] ROS 2 environment not found in /opt/ros!${NC}"
    exit 1
fi

# 2. Parse mode (default: effort / torque)
MODE="${1:-effort}"

case "$MODE" in
    torque|effort|tau)
        FIELD="effort"
        LABEL="Torque (Nm)"
        ;;
    pos|position|angle)
        FIELD="position"
        LABEL="Joint Position (rad)"
        ;;
    vel|velocity|speed)
        FIELD="velocity"
        LABEL="Joint Velocity (rad/s)"
        ;;
    *)
        echo -e "${YELLOW}[INFO] Unknown mode '$MODE'. Defaulting to effort (torque).${NC}"
        echo -e "Available modes: ${GREEN}effort${NC} (default), ${GREEN}pos${NC}, ${GREEN}vel${NC}"
        FIELD="effort"
        LABEL="Torque (Nm)"
        ;;
esac

echo -e "📈 Metric: ${GREEN}$LABEL${NC}"
echo -e "📡 Topic:  ${GREEN}/joint_states/$FIELD[0..11]${NC}"
echo -e "-----------------------------------------------------"
echo -e "Motor Index Legend:"
echo -e "  • BL Leg (can1): [0]=Collar, [1]=Hip, [2]=Knee"
echo -e "  • BR Leg (can0): [3]=Collar, [4]=Hip, [5]=Knee"
echo -e "  • FL Leg (can1): [6]=Collar, [7]=Hip, [8]=Knee"
echo -e "  • FR Leg (can0): [9]=Collar, [10]=Hip, [11]=Knee"
echo -e "-----------------------------------------------------"
echo -e "${YELLOW}Launching rqt_plot... (Press Ctrl+C in terminal to close)${NC}"

# 3. Assemble all 12 motor arguments
TOPICS=()
for i in {0..11}; do
    TOPICS+=("/joint_states/${FIELD}[$i]")
done

# 4. Launch rqt_plot
exec ros2 run rqt_plot rqt_plot "${TOPICS[@]}"
