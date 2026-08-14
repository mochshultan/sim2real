# 🐾 NXP Jaguar Quadruped: Sim-to-Real Deployment Guide
**Reinforcement Learning Control: Isaac Lab 3.0 (DreamWaQ) ➡️ RobStride RS00 Hardware via ROS**

---

## 📑 Daftar Isi
1. [Struktur & Dimensi State Observasi Policy (48-D)](#1-struktur--dimensi-state-observasi-policy-48-d)
2. [Tabel Pemetaan & Urutan Joint (Isaac Lab vs ROS Hardware)](#2-tabel-pemetaan--urutan-joint-isaac-lab-vs-ros-hardware)
3. [Parameter Aktuator & Gain Kontrol (RobStride RS00)](#3-parameter-aktuator--gain-kontrol-robstride-rs00)
4. [Arsitektur Sistem Sim-to-Real](#4-arsitektur-sistem-sim-to-real)
5. [SOP Prosedur Menjalankan Kode (Step-by-Step)](#5-sop-prosedur-menjalankan-kode-step-by-step)
6. [Fitur Keselamatan & E-Stop](#6-fitur-keselamatan--e-stop)

---

## 1. Struktur & Dimensi State Observasi Policy (48-D)

Model neural network (*actor policy*) yang dilatih pada **Isaac Lab 3.0** menerima vektor observasi berukuran tepat **48 dimensi** dengan urutan baku berikut:

| Rentang Indeks | Nama State | Dimensi | Satuan | Deskripsi & Rumus |
| :---: | :--- | :---: | :---: | :--- |
| `[0 : 3]` | **`base_lin_vel`** | 3 | $\text{m/s}$ | Kecepatan linier badan robot dalam *body frame* $[v_x, v_y, v_z]$ |
| `[3 : 6]` | **`base_ang_vel`** | 3 | $\text{rad/s}$ | Kecepatan sudut gyro dalam *body frame* $[\omega_x, \omega_y, \omega_z]$ |
| `[6 : 9]` | **`projected_gravity`** | 3 | unit | Proyeksi vektor gravitasi $[g_x, g_y, g_z]$ pada *body frame* |
| `[9 : 12]` | **`velocity_commands`** | 3 | $\text{m/s, rad/s}$ | Perintah joystick user $[v_x^{\text{cmd}}, v_y^{\text{cmd}}, \omega_z^{\text{cmd}}]$ |
| `[12 : 24]` | **`joint_pos_rel`** | 12 | $\text{rad}$ | Posisi sudut sendi relatif: $(q_i - q_{0, i})$ |
| `[24 : 36]` | **`joint_vel`** | 12 | $\text{rad/s}$ | Kecepatan sudut 12 sendi motor $\dot{q}_i$ |
| `[36 : 48]` | **`actions`** | 12 | $\text{rad}$ | Output aksi policy pada timestep sebelumnya $a_{t-1}$ (*last action*) |

$$\text{Total Dimensi Observasi} = 3 + 3 + 3 + 3 + 12 + 12 + 12 = \mathbf{48}$$

---

## 2. Tabel Pemetaan & Urutan Joint (Isaac Lab vs ROS Hardware)

Terdapat perbedaan pengelompokan indeks joint antara simulator Isaac Lab (dikelompokkan per jenis sendi) dan driver motor ROS CAN hardware (dikelompokkan per kaki).

### 📋 Tabel Perbandingan Indeks Joint

| Indeks Isaac Lab | Nama Joint (Isaac Lab) | Posisi Standby ($q_0$) | Indeks ROS | Nama Joint di ROS CAN (`parameters.py`) |
| :---: | :--- | :---: | :---: | :--- |
| **0** | `Fr_roll_joint` (Depan Kanan Roll) | $0.0\text{ rad}$ | **9** | `FR_collar_joint` |
| **1** | `Fl_roll_joint` (Depan Kiri Roll) | $0.0\text{ rad}$ | **6** | `FL_collar_joint` |
| **2** | `Br_roll_joint` (Belakang Kanan Roll) | $0.0\text{ rad}$ | **3** | `BR_collar_joint` |
| **3** | `Bl_roll_joint` (Belakang Kiri Roll) | $0.0\text{ rad}$ | **0** | `BL_collar_joint` |
| **4** | `Fr_hip_pitch_joint` (Depan Kanan Hip) | $-1.5\text{ rad}$ | **10** | `FR_hip_joint` |
| **5** | `Fl_hip_pitch_joint` (Depan Kiri Hip) | $-1.5\text{ rad}$ | **7** | `FL_hip_joint` |
| **6** | `Br_hip_pitch_joint` (Belakang Kanan Hip) | $-1.5\text{ rad}$ | **4** | `BR_hip_joint` |
| **7** | `Bl_hip_pitch_joint` (Belakang Kiri Hip) | $-1.5\text{ rad}$ | **1** | `BL_hip_joint` |
| **8** | `Fr_knee_joint` (Depan Kanan Lutut) | $+1.5\text{ rad}$ | **11** | `FR_knee_joint` |
| **9** | `Fl_knee_joint` (Depan Kiri Lutut) | $+1.5\text{ rad}$ | **8** | `FL_knee_joint` |
| **10** | `Br_knee_joint` (Belakang Kanan Lutut) | $+1.5\text{ rad}$ | **5** | `BR_knee_joint` |
| **11** | `Bl_knee_joint` (Belakang Kiri Lutut) | $+1.5\text{ rad}$ | **2** | `BL_knee_joint` |

### 🔄 Formula Konversi Remapping Indeks:
```python
# Konversi data dari ROS ke Isaac Lab:
ROS_TO_ISAAC = [9, 6, 3, 0, 10, 7, 4, 1, 11, 8, 5, 2]

# Konversi output aksi dari Isaac Lab kembali ke urutan motor ROS CAN:
ISAAC_TO_ROS = [3, 7, 11, 2, 6, 10, 1, 5, 9, 0, 4, 8]
```

---

## 3. Parameter Aktuator & Gain Kontrol (RobStride RS00)

| Parameter | Nilai | Keterangan |
| :--- | :---: | :--- |
| **Frekuensi Kontrol (*Control Rate*)** | **`50 Hz`** | $\Delta t = 0.02\text{ s}$ ($20\text{ ms}$) |
| **Action Scale** | **`0.25`** | $q_{\text{des}} = q_0 + 0.25 \times a_{\text{policy}}$ |
| **Stiffness Gain ($K_p$)** | **`25.0`** | Kekakuan sendi posisi PD |
| **Damping Gain ($K_d$)** | **`1.0`** | Redaman kecepatan sendi PD |
| **Torsi Maksimal RS00** | **`14.0 ~ 17.0 Nm`** | Batas aman motor QDD RobStride RS00 |
| **Batas Sudut Lutut (*Knee Limits*)** | `[-0.1, 2.8] rad` | Batas mekanis empat batang linkage |
| **Batas Sudut Roll (*Roll Limits*)** | `[-0.4, 0.4] rad` | Batas ayunan bahu |

---

## 4. Arsitektur Sistem Sim-to-Real

```
                ┌──────────────────────────────────────────────┐
                │      Joystick Teleop / Navigation Stack      │
                │        /joy  atau  /cmd_vel (Odometry)       │
                └──────────────────────┬───────────────────────┘
                                       │
                                       ▼
┌──────────────────┐    ┌──────────────────────────────────────┐    ┌──────────────────┐
│     IMU Sensor   │───>│   NXP Jaguar Real Controller Node    │───>│ CAN Bus Driver   │
│    (/imu/data)   │    │      (nxp_jaguar_controller.py)      │    │  (can0 / can1)   │
└──────────────────┘    │                                      │    └────────┬─────────┘
┌──────────────────┐    │  • Bangun Vektor 48-D                │             │
│ Joint Encoders   │───>│  • Inferensi policy.pt (50 Hz JIT)   │             ▼
│  (/joint_states) │    │  • Remapping Indeks ROS <-> Isaac    │    ┌──────────────────┐
└──────────────────┘    │  • E-Stop Tilt Protection (> 60°)    │    │ 12x Motor RS00   │
                        └──────────────────────────────────────┘    └──────────────────┘
```

---

## 5. SOP Prosedur Menjalankan Kode (Step-by-Step)

### Persiapan File:
Pastikan file model TorchScript JIT terbaru telah berada di folder `models/`:
```bash
cp /home/shultan/IsaacLab/logs/rsl_rl/nxp_jaguar_rough/2026-08-14_12-10-56/exported/policy.pt \
   /home/shultan/mevius2_ws_ros-o/src/mevius2-master/models/policy.pt
```

---

### 🚀 Urutan Eksekusi Terminal (SOP Deployment):

#### 🔹 Terminal 1: Jalankan ROS Core
```bash
roscore
```

#### 🔹 Terminal 2: Inisialisasi Antarmuka CAN Bus Motor (Hardware)
Jalankan script `bringup_canbus.sh` (sudah mencakup konfigurasi `bitrate 1000000` dan `txqueuelen 1000` untuk transmisi data motor kecepatan tinggi tanpa packet drop):
```bash
sudo ./scripts/bringup_canbus.sh
```

*Atau via perintah manual:*
```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 txqueuelen 1000
sudo ip link set can0 up

sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 txqueuelen 1000
sudo ip link set can1 up
```

#### 🔹 Terminal 3: Jalankan Driver IMU & Sensor
```bash
# Jalankan node driver IMU robot
roslaunch mevius2 imu.launch
```

#### 🔹 Terminal 4: Jalankan Driver Motor CAN & Joystick
```bash
# Jalankan node pembacaan CAN motor & teleop joystick
roslaunch mevius2 hardware.launch
```

#### 🔹 Terminal 5: Jalankan Controller Utama Sim-to-Real NXP Jaguar
```bash
rosrun mevius2 nxp_jaguar_controller.py
```

---

### 🕹️ State Machine Pengoperasian Robot:

1. **Kondisi Awal (`STANDBY`)**:
   - Robot dalam posisi duduk/istirahat. Motor aktif mempertahankan posisi diam dengan aman.
2. **Berdiri (`STANDUP`)**:
   - Tekan **Tombol X (Cross)** pada joystick. Robot perlahan berdiri ke sudut nominal $q_0$ (Roll: 0, Hip: -1.5, Knee: +1.5).
3. **Mulai Berjalan (`WALK`)**:
   - Tekan **Tombol Lingkaran (Circle)** saat robot sudah dalam posisi `STANDUP`.
   - Model RL `policy.pt` aktif memproses input sensor 50 Hz.
   - Gerakkan **Stik Analog Kiri** untuk maju/mundur/geser dan **Stik Analog Kanan** untuk berputar (*yaw*).

---

## 6. Fitur Keselamatan & E-Stop

1. **Auto E-Stop Kemiringan (*Tilt Protection*)**:
   - Jika robot miring lebih dari **$60^\circ$** (terjatuh / terbalik), controller akan secara otomatis mematikan perintah gerak (*gains drop*) dan kembali ke state `STANDBY`.
2. **Proteksi Hilang Koneksi Sensor (*Sensor Timeout Guard*)**:
   - Jika data IMU atau encoder terputus $> 100\text{ ms}$, robot otomatis menghentikan aksi.
3. **Uji Coba Gantung (*Suspended Bench Test*)**:
   - Selalu lakukan pengetesan pertama dengan robot digantung di udara sebelum ditaruh di lantai fisik.
