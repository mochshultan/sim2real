# ⚙️ Utility Binaries & Setup Scripts (`bin/`)

This directory contains system-level configuration scripts for hardware setup.

---

## 📄 Scripts

### [`can_setup.sh`](./can_setup.sh)
Automates USB-CAN adapter discovery using `udevadm`, resolves device interface symlinks (`/dev/can_usb_0` and `/dev/can_usb_1`), renames interfaces systematically to `can0` and `can1`, sets bitrate to 1 Mbps, and brings up the SocketCAN interfaces.
