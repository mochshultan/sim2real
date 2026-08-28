#!/usr/bin/env bash
# ==============================================================================
# 🐾 NXP Jaguar: SocketCAN Bus Initialization (can0 & can1 @ 1 Mbps)
# ==============================================================================

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${GREEN}=====================================================${NC}"
echo -e "${GREEN} 🐾 NXP JAGUAR: BRINGUP SOCKETCAN BUS (1 Mbps)       ${NC}"
echo -e "${GREEN}=====================================================${NC}"

# Check for root / sudo privileges
if [ "$EUID" -ne 0 ]; then
  echo -e "${YELLOW}[INFO] Running with sudo privileges...${NC}"
  SUDO="sudo"
else
  SUDO=""
fi

for iface in can0 can1; do
  echo -e "Configuring interface ${GREEN}$iface${NC}..."
  $SUDO ip link set "$iface" down 2>/dev/null || true
  $SUDO ip link set "$iface" type can bitrate 1000000
  $SUDO ip link set "$iface" txqueuelen 1000
  $SUDO ip link set "$iface" up
  echo -e "  ✅ $iface is UP (1,000,000 bps, txqueuelen 1000)"
done

echo -e "\n${GREEN}📊 Current CAN Interfaces Status:${NC}"
ip -brief link show can0 can1
echo -e "\n${GREEN}✅ CAN bus initialization complete.${NC}"

