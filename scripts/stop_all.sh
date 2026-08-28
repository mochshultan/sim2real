#!/bin/bash
# ==============================================================================
# Script Teardown / Stop All: Mematikan proses robot, ROS 2, dan SocketCAN
# ==============================================================================

echo "=========================================="
echo "🛑 Menghentikan Proses Sim2Real & ROS 2..."
echo "=========================================="

# Daftar pola proses robot yang akan dihentikan
TARGETS=(
  "can_hardware_node.py"
  "nxp_jaguar_controller.py"
  "check_states.py"
  "check_joints.py"
  "keyboard_teleop.py"
  "remote_xbox_forwarder.py"
  "test_sit_stand.py"
  "scan_robostride_ids.py"
  "set_robostride_zero.py"
  "virtual_gamepad"
  "bringup_imu.sh"
  "bringup_canbus.sh"
  "livox"
)

found_any=false
for target in "${TARGETS[@]}"; do
  if pgrep -f "$target" > /dev/null 2>&1; then
    echo "  [x] Menghentikan proses: $target"
    pkill -9 -f "$target" 2>/dev/null || true
    found_any=true
  fi
done

if [ "$found_any" = false ]; then
  echo "  [i] Tidak ada proses script robot yang sedang berjalan."
fi

# Hentikan ROS 2 daemon jika ada
if command -v ros2 >/dev/null 2>&1; then
  echo "  [x] Menghentikan ROS 2 daemon..."
  ros2 daemon stop >/dev/null 2>&1 || true
fi

echo ""
echo "=========================================="
echo "🔌 Mematikan Interface CAN (can0 & can1)..."
echo "=========================================="

for iface in can0 can1; do
  if ip link show "$iface" > /dev/null 2>&1; then
    echo "  [v] Mematikan $iface..."
    sudo ip link set "$iface" down 2>/dev/null || true
  else
    echo "  [-] Interface $iface tidak ditemukan."
  fi
done

echo ""
echo "=========================================="
echo "📊 Status CAN Interface:"
echo "=========================================="
ip -brief link show can0 2>/dev/null || echo "can0: not found"
ip -brief link show can1 2>/dev/null || echo "can1: not found"

echo ""
echo "✅ Selesai! Semua proses telah dihentikan dan interface CAN dimatikan."
