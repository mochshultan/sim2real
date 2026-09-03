# Setup Scripts (`bin/`)

System configuration scripts for hardware bringup.

## Scripts

### [`can_setup.sh`](./can_setup.sh)
Detects USB-CAN adapters with `udevadm`, maps symlinks (`/dev/can_usb_0` and `/dev/can_usb_1`) to `can0` and `can1`, sets the bitrate to 1 Mbps, and brings up both SocketCAN interfaces.
