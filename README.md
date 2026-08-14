# 🐾 NXP Jaguar Sim-to-Real Deployment & ROS 2 Control Framework
**Reinforcement Learning Control: Isaac Lab 3.0 (DreamWaQ) ➡️ RobStride RS00 Hardware via ROS 2**

Repository ini berisi framework lengkap untuk deployment model kontrol Reinforcement Learning (RL) hasil pelatihan **Isaac Lab 3.0** ke hardware fisik robot quadruped **NXP Jaguar** menggunakan **ROS 2**.

---

## 📑 Daftar Isi
1. [Prasyarat & Versi ROS yang Dibutuhkan](#1-prasyarat--versi-ros-yang-dibutuhkan)
2. [Tabel Urutan State Observasi Policy (48 Dimensi)](#2-tabel-urutan-state-observasi-policy-48-dimensi)
3. [Tabel Pemetaan & Urutan Joint (Isaac Lab vs ROS Hardware)](#3-tabel-pemetaan--urutan-joint-isaac-lab-vs-ros-hardware)
4. [Tutorial Diagnostik & Verifikasi State Robot](#4-tutorial-diagnostik--verifikasi-state-robot)
5. [SOP Prosedur Deployment Hardware (Langkah Demi Langkah)](#5-sop-prosedur-deployment-hardware-langkah-demi-langkah)
6. [Fitur Keselamatan & E-Stop](#6-fitur-keselamatan--e-stop)

---

## 1. Prasyarat & Versi ROS yang Dibutuhkan

Repository ini dirancang dan dioptimalkan secara murni untuk **ROS 2**:

- **OS Target**: Ubuntu 24.04 LTS atau Ubuntu 22.04 LTS
- **Versi ROS 2 yang Didukung**:
  - 🟢 **ROS 2 Jazzy Jalisco** (Rekomendasi untuk Ubuntu 24.04)
  - 🟢 **ROS 2 Humble Hawksbill** (Untuk Ubuntu 22.04)
  - 🟢 **ROS 2 Iron Irwini**
- **Python Dependencies**:
  - `torch >= 2.0.0` (Untuk inferensi model JIT `policy.pt`)
  - `numpy`
  - `python-can` (Untuk driver komunikasi CAN Bus RS00)

```bash
# Source environment ROS 2 (contoh Jazzy):
source /opt/ros/jazzy/setup.bash
```

---

## 2. Tabel Urutan State Observasi Policy (48 Dimensi)

Model neural network (*actor policy*) yang diekspor dari Isaac Lab menerima vektor observasi berukuran tepat **48 dimensi** dengan urutan baku berikut:

| Rentang Indeks | Nama State | Dimensi | Satuan | Deskripsi & Rumus |
| :---: | :--- | :---: | :---: | :--- |
| `[0 : 3]` | **`base_lin_vel`** | 3 | $\text{m/s}$ | Kecepatan linier badan robot dalam *body frame* $[v_x, v_y, v_z]$ |
| `[3 : 6]` | **`base_ang_vel`** | 3 | $\text{rad/s}$ | Kecepatan sudut gyro dalam *body frame* $[\omega_x, \omega_y, \omega_z]$ |
| `[6 : 9]` | **`projected_gravity`** | 3 | unit | Proyeksi vektor gravitasi $[g_x, g_y, g_z]$ pada *body frame* ($[0, 0, 1]$ saat tegak) |
| `[9 : 12]` | **`velocity_commands`** | 3 | $\text{m/s, rad/s}$ | Perintah joystick user $[v_x^{\text{cmd}}, v_y^{\text{cmd}}, \omega_z^{\text{cmd}}]$ |
| `[12 : 24]` | **`joint_pos_rel`** | 12 | $\text{rad}$ | Posisi sudut sendi relatif terhadap nominal: $(q_i - q_{0, i})$ |
| `[24 : 36]` | **`joint_vel`** | 12 | $\text{rad/s}$ | Kecepatan sudut 12 sendi motor $\dot{q}_i$ |
| `[36 : 48]` | **`actions`** | 12 | $\text{rad}$ | Output aksi policy pada timestep sebelumnya $a_{t-1}$ (*last action*) |

$$\text{Total Dimensi Observasi} = 3 + 3 + 3 + 3 + 12 + 12 + 12 = \mathbf{48}$$

---

## 3. Tabel Pemetaan & Urutan Joint (Isaac Lab vs ROS Hardware)

Terdapat perbedaan pengelompokan indeks sendi antara simulator **Isaac Lab** (dikelompokkan per jenis sendi: semua Roll, semua Hip, semua Knee) dan driver motor **ROS CAN Hardware** (dikelompokkan per kaki: BL, BR, FL, FR).

### 📋 Tabel Perbandingan & Urutan Joint:

| Indeks Isaac Lab | Nama Joint (Isaac Lab) | Posisi Standby ($q_0$) | Indeks ROS | Nama Joint di ROS CAN (`parameters.py`) | ID Motor | CAN Bus |
| :---: | :--- | :---: | :---: | :--- | :---: | :---: |
| **0** | `Fr_roll_joint` (Depan Kanan Roll) | $0.0\text{ rad}$ | **9** | `FR_collar_joint` | 1 | `can1` |
| **1** | `Fl_roll_joint` (Depan Kiri Roll) | $0.0\text{ rad}$ | **6** | `FL_collar_joint` | 4 | `can1` |
| **2** | `Br_roll_joint` (Belakang Kanan Roll) | $0.0\text{ rad}$ | **3** | `BR_collar_joint` | 7 | `can0` |
| **3** | `Bl_roll_joint` (Belakang Kiri Roll) | $0.0\text{ rad}$ | **0** | `BL_collar_joint` | 10 | `can0` |
| **4** | `Fr_hip_pitch_joint` (Depan Kanan Hip) | $-1.5\text{ rad}$ | **10** | `FR_hip_joint` | 2 | `can1` |
| **5** | `Fl_hip_pitch_joint` (Depan Kiri Hip) | $-1.5\text{ rad}$ | **7** | `FL_hip_joint` | 5 | `can1` |
| **6** | `Br_hip_pitch_joint` (Belakang Kanan Hip) | $-1.5\text{ rad}$ | **4** | `BR_hip_joint` | 8 | `can0` |
| **7** | `Bl_hip_pitch_joint` (Belakang Kiri Hip) | $-1.5\text{ rad}$ | **1** | `BL_hip_joint` | 11 | `can0` |
| **8** | `Fr_knee_joint` (Depan Kanan Lutut) | $+1.5\text{ rad}$ | **11** | `FR_knee_joint` | 3 | `can1` |
| **9** | `Fl_knee_joint` (Depan Kiri Lutut) | $+1.5\text{ rad}$ | **8** | `FL_knee_joint` | 6 | `can1` |
| **10** | `Br_knee_joint` (Belakang Kanan Lutut) | $+1.5\text{ rad}$ | **5** | `BR_knee_joint` | 9 | `can0` |
| **11** | `Bl_knee_joint` (Belakang Kiri Lutut) | $+1.5\text{ rad}$ | **2** | `BL_knee_joint` | 12 | `can0` |

### 🔄 Formula Remapping di Node Python:
```python
# Konversi urutan ROS CAN (BL, BR, FL, FR) ke urutan Isaac Lab:
ROS_TO_ISAAC = [9, 6, 3, 0, 10, 7, 4, 1, 11, 8, 5, 2]

# Konversi output aksi Isaac Lab kembali ke urutan ROS CAN:
ISAAC_TO_ROS = [3, 7, 11, 2, 6, 10, 1, 5, 9, 0, 4, 8]
```

---

## 4. Tutorial Diagnostik & Verifikasi State Robot

Sebelum menjalankan kontrol autonomous, Anda **WAJIB** memverifikasi bahwa sensor IMU dan encoder 12 sendi motor terbaca dengan arah tanda (*sign*) dan urutan yang benar.

### 🔍 Tool 1: Live State & Observation Dashboard (`check_states.py`)
Jalankan tool diagnostik untuk melihat tabel real-time 48 dimensi state:
```bash
python3 scripts/check_states.py
```
**Tampilan yang dihasilkan:**
```text
================================================================================
 🐾 NXP JAGUAR: 48-D ACTOR OBSERVATION STATE & JOINT DIAGNOSTIC DASHBOARD
================================================================================
📡 SENSOR STATUS | IMU Msg:   1240 | Joint Msg:   1240
--------------------------------------------------------------------------------
1. BASE VELOCITY & GRAVITY PROJECTION:
   • Base Lin Vel  [0:3] : [ +0.00,  +0.00,  +0.00] m/s
   • Base Ang Vel  [3:6] : [ +0.00,  +0.00,  +0.00] rad/s
   • Proj Gravity  [6:9] : [ +0.00,  +0.00,  +1.00] (Upright should be [0.0, 0.0, 1.0])
   • Velocity Cmd [9:12] : [ +0.00,  +0.00,  +0.00]
--------------------------------------------------------------------------------
2. 12-JOINT STATE ORDER (Isaac Lab Actor Dimension [12:36]):
   Index  Joint Name (Isaac Lab)   q_curr (rad)   q0_nom     q - q0 (rel)   q_dot (rad/s) 
   ----------------------------------------------------------------------------
   [00]   Fr_roll_joint                 +0.0012      +0.00        +0.0012        +0.0000
   [01]   Fl_roll_joint                 -0.0008      +0.00        -0.0008        +0.0000
   [02]   Br_roll_joint                 +0.0005      +0.00        +0.0005        +0.0000
   [03]   Bl_roll_joint                 -0.0002      +0.00        -0.0002        +0.0000
   [04]   Fr_hip_pitch_joint            -1.4980      -1.50        +0.0020        +0.0000
   [05]   Fl_hip_pitch_joint            -1.5010      -1.50        -0.0010        +0.0000
   [06]   Br_hip_pitch_joint            -1.4995      -1.50        +0.0005        +0.0000
   [07]   Bl_hip_pitch_joint            -1.5002      -1.50        -0.0002        +0.0000
   [08]   Fr_knee_joint                 +1.5020      +1.50        +0.0020        +0.0000
   [09]   Fl_knee_joint                 +1.4985      +1.50        -0.0015        +0.0000
   [10]   Br_knee_joint                 +1.5010      +1.50        +0.0010        +0.0000
   [11]   Bl_knee_joint                 +1.4990      +1.50        -0.0010        +0.0000
--------------------------------------------------------------------------------
3. AUTOMATED SANITY CHECKS:
   [OK] IMU Stream Active: Received
   [OK] Joint State Stream: Received 12 joints
   [OK] Robot Orientation: Upright
   [OK] Max Deviation from Stand Pose: 0.002 rad
================================================================================
```

---

## 5. SOP Prosedur Deployment Hardware (Langkah Demi Langkah)

Ikuti urutan Standar Operasional Prosedur (SOP) berikut untuk mencegah resiko kerusakan hardware:

### 🟡 Tahap 1: Persiapan Mekanik & Gantung Robot
1. **Gantung Robot pada Tali Pengaman (*Gantry / Rig Hanging*)**:
   - Ikatkan tali penggantung pada titik pusat badan robot (*center of mass*).
   - Pastikan ke-4 kaki robot **menggantung bebas di udara** (tidak menyentuh lantai/meja).
2. **Siapkan Tombol E-Stop Fisik / Remote Controller**.

---

### 🟢 Tahap 2: Power-Up & Inisialisasi Bus CAN
1. **Pasang & Nyalakan Baterai**:
   - Hubungkan baterai LiPo (24V / 6S) ke power distribution board robot.
   - Nyalakan saklar daya utama motor RS00.
2. **Inisialisasi Antarmuka CAN Bus**:
   Buka terminal di PC robot (on-board Jetson / Mini PC) dan jalankan:
   ```bash
   cd ~/mevius2_ws_ros-o/src/mevius2-master
   sudo ./scripts/bringup_canbus.sh
   ```
   *(Script ini otomatis mengonfigurasi `can0` dan `can1` dengan `bitrate 1000000` dan `txqueuelen 1000`)*.

---

### 🔵 Tahap 3: Menjalankan Node Driver & Sensor (ROS 2)

Buka 3 terminal terpisah:

#### Terminal 1: Jalankan Driver IMU
```bash
source /opt/ros/jazzy/setup.bash
# Jalankan node pembacaan sensor IMU robot (/imu/data)
ros2 run jaguar_control imu_node
```

#### Terminal 2: Jalankan Driver CAN Motor RS00 & Joystick
```bash
source /opt/ros/jazzy/setup.bash
# Jalankan driver motor CAN 200 Hz
python3 scripts/xiaomimotor_test.py
```

#### Terminal 3: Verifikasi State Sensor
```bash
python3 scripts/check_states.py
```
*(Pastikan semua status bertanda `[OK]` dan gravitasi terbaca $[0, 0, 1]$)*.

---

### 🟣 Tahap 4: Menjalankan Controller Utama Sim-to-Real NXP Jaguar

#### Terminal 4: Jalankan Node RL Controller
```bash
source /opt/ros/jazzy/setup.bash
python3 scripts/nxp_jaguar_controller.py
```

---

### 🕹️ Tahap 5: Prosedur Pengoperasian Joystick (State Machine)

1. **Kondisi Awal (`STANDBY`)**:
   - Robot dalam posisi duduk/diam aman. Motor menahan posisi standby dengan torsi rendah.
2. **Berdiri (`STANDUP`)**:
   - Tekan **Tombol X (Cross)** pada joystick.
   - Robot perlahan mengangkat badan dan berdiri kokoh pada sudut nominal $q_0$.
3. **Mulai Berjalan (`WALK`)**:
   - Tekan **Tombol Lingkaran (Circle)** saat robot sudah berdiri tegak.
   - Policy RL `policy.pt` aktif memproses observasi 50 Hz.
   - **Stik Analog Kiri**: Dorong ke depan/belakang untuk maju/mundur ($v_x$), geser kiri/kanan untuk strafe ($v_y$).
   - **Stik Analog Kanan**: Dorong kiri/kanan untuk manuver berputar (*yaw* $\omega_z$).
4. **Uji Coba Lantai Fisik**:
   - Setelah semua respon di udara terbukti benar, turunkan robot secara perlahan ke lantai datar dan lakukan uji jalan bertahap.

---

## 6. Fitur Keselamatan & E-Stop

1. **Auto E-Stop Kemiringan (*Tilt Protection*)**:
   - Jika robot miring lebih dari **$60^\circ$** (terjatuh / terbalik), controller otomatis memutus torsi gerak dan kembali ke state `STANDBY`.
2. **Emergency Cutoff Joystick**:
   - Tekan **Tombol Kotak (Square)** kapan saja untuk langsung memaksa robot kembali ke state `STANDBY`.
3. **Loss of Signal Protection**:
   - Jika sinyal IMU atau CAN terputus $> 100\text{ ms}$, loop kontrol otomatis mengunci posisi aman.
