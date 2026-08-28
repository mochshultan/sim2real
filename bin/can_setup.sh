#!/usr/bin/env bash
# ==============================================================================
# 🐾 NXP Jaguar: USB-CAN Adapter Enumeration and Interface Renaming
# Maps /dev/can_usb_0 -> can0 and /dev/can_usb_1 -> can1 based on udev rules.
# ==============================================================================

set -e

temp0="can_temp_0"
temp1="can_temp_1"

# Query the network interface name linked to /dev/can_usb_*
get_iface() {
  dev_path=$(udevadm info -q path -n "/dev/can_usb_$1" 2>/dev/null)
  find /sys/class/net -lname "*$dev_path*" -exec basename {} \; 2>/dev/null
}

iface0=$(get_iface 0)
iface1=$(get_iface 1)

if [ -z "$iface0" ] || [ -z "$iface1" ]; then
  echo "[ERROR] Could not identify one or both USB-CAN interfaces (/dev/can_usb_0, /dev/can_usb_1)."
  echo "        Ensure USB-CAN adapters are plugged in and 99-usb-can.rules is installed."
  exit 1
fi

echo "Found CAN interfaces: iface0=$iface0, iface1=$iface1"

# Bring down interfaces before renaming
for iface in "$iface0" "$iface1"; do
  sudo ip link set "$iface" down || true
done

# Temporary rename to prevent naming collisions
if [ "$iface0" = "can1" ] || [ "$iface0" = "can0" ]; then
  sudo ip link set "$iface0" name "$temp0"
  iface0="$temp0"
fi
if [ "$iface1" = "can0" ] || [ "$iface1" = "can1" ]; then
  sudo ip link set "$iface1" name "$temp1"
  iface1="$temp1"
fi

# Apply final standardized interface names
sudo ip link set "$iface0" name "can0"
sudo ip link set "$iface1" name "can1"

# Configure CAN bitrate and bring up
for iface in can0 can1; do
  sudo ip link set "$iface" type can bitrate 1000000
  sudo ip link set "$iface" txqueuelen 1000
  sudo ip link set "$iface" up
  echo "  ✅ $iface configured @ 1 Mbps (UP)"
done

echo "CAN bus setup completed successfully."


