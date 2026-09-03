# Configuration Files (`config/`)

Hardware rules, sensor parameters, SLAM configuration, and RViz profiles for the NXP Jaguar quadruped.

## 1. Hardware and Network Rules

| File | Format | Description |
| :--- | :---: | :--- |
| [`99-usb-can.rules`](./99-usb-can.rules) | `udev` | Maps USB-CAN adapters by serial number to `/dev/can_usb_0` (`can0`) and `/dev/can_usb_1` (`can1`). |
| [`MID360_config.json`](./MID360_config.json) | `JSON` | Livox MID-360 LiDAR network parameters: IP addresses, broadcast codes, and data ports. |

## 2. SLAM and Elevation Mapping

| File | Format | Description |
| :--- | :---: | :--- |
| [`mid360_fastlio_config.yaml`](./mid360_fastlio_config.yaml) | `YAML` | FAST-LIO LiDAR-inertial odometry configuration: extrinsic matrices, IMU covariance, and map resolution. |
| [`elevation_core.yaml`](./elevation_core.yaml) | `YAML` | CuPy elevation grid parameters: cell size, decay rate, and map resolution. |
| [`elevation_jaguar.yaml`](./elevation_jaguar.yaml) | `YAML` | Robot footprint dimensions and sensor frame links for elevation mapping. |
| [`c1_camera_info.yaml`](./c1_camera_info.yaml) | `YAML` | Intrinsic calibration matrix and distortion coefficients for the onboard camera. |

## 3. RViz Profiles

| File | Format | Description |
| :--- | :---: | :--- |
| [`jaguar_display.rviz`](./jaguar_display.rviz) | `RViz` | Displays robot model with TF transforms and joint states. |
| [`jaguar_display_livox.rviz`](./jaguar_display_livox.rviz) | `RViz` | Displays robot model with Livox LiDAR point clouds. |
| [`simple_grid.rviz`](./simple_grid.rviz) | `RViz` | Displays 2.5D elevation grid map. |
