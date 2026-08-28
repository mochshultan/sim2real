# ⚙️ Configuration Files Reference (`config/`)

This directory contains hardware device rules, sensor configurations, SLAM parameters, and RViz display profiles for the **NXP Jaguar Quadruped**.

---

## 📑 Configuration Files Summary

### 🔌 1. Hardware & Network Rules
| File | Format | Description |
| :--- | :---: | :--- |
| [`99-usb-can.rules`](./99-usb-can.rules) | `udev` | Linux udev rule mapping USB-CAN adapters by serial number to `/dev/can_usb_0` (can0) and `/dev/can_usb_1` (can1). |
| [`MID360_config.json`](./MID360_config.json) | `JSON` | Livox MID-360 LiDAR Ethernet network configuration (IP addresses, broadcast codes, and data ports). |

---

### 🗺️ 2. SLAM & Elevation Mapping
| File | Format | Description |
| :--- | :---: | :--- |
| [`mid360_fastlio_config.yaml`](./mid360_fastlio_config.yaml) | `YAML` | FAST-LIO LiDAR-inertial odometry configuration (extrinsic matrices, IMU covariance, and map resolution). |
| [`elevation_core.yaml`](./elevation_core.yaml) | `YAML` | GPU-accelerated CuPy elevation mapping grid parameters (cell size, decay, resolution). |
| [`elevation_jaguar.yaml`](./elevation_jaguar.yaml) | `YAML` | NXP Jaguar robot footprint geometry and sensor frame links for elevation mapping. |
| [`c1_camera_info.yaml`](./c1_camera_info.yaml) | `YAML` | Intrinsic calibration parameters and distortion coefficients for onboard camera. |

---

### 🖥️ 3. RViz Visualizer Profiles
| File | Format | Description |
| :--- | :---: | :--- |
| [`jaguar_display.rviz`](./jaguar_display.rviz) | `RViz` | Standard robot model display with TF transforms and joint states. |
| [`jaguar_display_livox.rviz`](./jaguar_display_livox.rviz) | `RViz` | Robot model combined with real-time Livox LiDAR point clouds. |
| [`simple_grid.rviz`](./simple_grid.rviz) | `RViz` | 2.5D elevation grid map visualization. |
