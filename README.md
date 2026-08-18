# 🐾 NXP Jaguar Quadruped: Sim-to-Real Deployment & ROS 2 Control Framework
**Reinforcement Learning Control: Isaac Lab 3.0 (DreamWaQ) ➡️ RobStride RS00 Hardware via ROS 2**

Framework lengkap untuk deployment model kontrol Reinforcement Learning (RL) hasil pelatihan **Isaac Lab 3.0** ke hardware fisik robot quadruped **NXP Jaguar** menggunakan **ROS 2**.

---

## 📑 Daftar Isi
1. [Prasyarat & Lingkungan Sistem](#1-prasyarat--lingkungan-sistem)
2. [Struktur & Dimensi State Observasi Policy (48 Dimensi)](#2-struktur--dimensi-state-observasi-policy-48-dimensi)
3. [Verifikasi & Mekanisme Remapping State Robot ke Policy](#3-verifikasi--mekanisme-remapping-state-robot-ke-policy)
4. [Tabel Pemetaan & Urutan Joint (Isaac Lab vs ROS Hardware)](#4-tabel-pemetaan--urutan-joint-isaac-lab-vs-ros-hardware)
5. [Parameter Aktuator & Gain Kontrol (RobStride RS00)](#5-parameter-aktuator--gain-kontrol-robstride-rs00)
6. [Konfigurasi & Panduan Joystick (Hardware & Virtual)](#6-konfigurasi--panduan-joystick-hardware--virtual)
7. [SOP Prosedur Deployment Hardware (Langkah Demi Langkah)](#7-sop-prosedur-deployment-hardware-langkah-demi-langkah)
8. [Fitur Keselamatan & E-Stop](#8-fitur-keselamatan--e-stop)
9. [Validasi & Visualisasi MuJoCo Sim-to-Sim](#9-validasi--visualisasi-mujoco-sim-to-sim)

---

## 1. Prasyarat & Lingkungan Sistem

- **Sistem Operasi**: Ubuntu 22.04 LTS (x86_64 / Intel N150)
- **Distribusi ROS 2**: ROS 2 Humble Hawksbill (`/opt/ros/humble`)
- **Python Dependencies**:
  - `torch >= 2.0.0` (PyTorch CPU untuk inferensi model JIT `policy.pt`)
  - `onnxruntime`
  - `python-can` (SocketCAN driver)
  - `numpy`, `scipy`

---

## 2. Struktur & Dimensi State Observasi Policy (48 Dimensi)

Model neural network (*actor policy*) yang diekspor dari Isaac Lab menerima vektor observasi berukuran tepat **48 dimensi**:

| Rentang Indeks | Nama State | Dimensi | Satuan | Deskripsi & Rumus |
| :---: | :--- | :---: | :---: | :--- |
| `[0 : 3]` | **`base_lin_vel`** | 3 | $\text{m/s}$ | Kecepatan linier badan robot dalam *body frame* $[v_x, v_y, v_z]$ |
| `[3 : 6]` | **`base_ang_vel`** | 3 | $\text{rad/s}$ | Kecepatan sudut gyro dalam *body frame* $[\omega_x, \omega_y, \omega_z]$ |
| `[6 : 9]` | **`projected_gravity`** | 3 | unit | Proyeksi vektor gravitasi $[g_x, g_y, g_z]$ ($[0, 0, -1]$ saat tegak) |
| `[9 : 12]` | **`velocity_commands`** | 3 | $\text{m/s, rad/s}$ | Perintah joystick user $[v_x^{\text{cmd}}, v_y^{\text{cmd}}, \omega_z^{\text{cmd}}]$ |
| `[12 : 24]` | **`joint_pos_rel`** | 12 | $\text{rad}$ | Posisi sudut sendi relatif terhadap nominal: $(q_i - q_{0, i})$ |
| `[24 : 36]` | **`joint_vel`** | 12 | $\text{rad/s}$ | Kecepatan sudut 12 sendi motor $\dot{q}_i$ |
| `[36 : 48]` | **`actions`** | 12 | $\text{rad}$ | Output aksi policy pada timestep sebelumnya $a_{t-1}$ (*last action*) |

$$\text{Total Dimensi Observasi} = 3 + 3 + 3 + 3 + 12 + 12 + 12 = \mathbf{48}$$

---

## 3. Verifikasi & Mekanisme Remapping State Robot ke Policy

### ⚠️ Mengapa Remapping Wajib Dilakukan?
Terdapat perbedaan fundamental dalam urutan pengelompokan joint antara **Hardware Driver CAN** dan **Isaac Lab RL Policy**:
- **Driver Hardware CAN**: Mengelompokkan sendi **per kaki** (`BL`, `BR`, `FL`, `FR`).
- **Policy Isaac Lab**: Mengelompokkan sendi **per tipe sendi** (semua `Roll`, semua `Hip`, semua `Knee`).

```
[Driver Motor CAN Hardware]                  [Model RL Policy Isaac Lab 3.0]
Dikelompokkan per KAKI (BL, BR, FL, FR)      Dikelompokkan per TIPE SENDI (Rolls, Hips, Knees)
─────────────────────────────────────        ────────────────────────────────────────────────
 0: BL_collar_joint                           0: Fr_roll_joint   (Roll Depan Kanan)
 1: BL_hip_joint                              1: Fl_roll_joint   (Roll Depan Kiri)
 2: BL_knee_joint                             2: Br_roll_joint   (Roll Belakang Kanan)
 3: BR_collar_joint                           3: Bl_roll_joint   (Roll Belakang Kiri)
 4: BR_hip_joint                              4: Fr_hip_pitch    (Hip Depan Kanan)
 5: BR_knee_joint                             5: Fl_hip_pitch    (Hip Depan Kiri)
 6: FL_collar_joint                           6: Br_hip_pitch    (Hip Belakang Kanan)
 7: FL_hip_joint                              7: Bl_hip_pitch    (Hip Belakang Kiri)
 8: FL_knee_joint                             8: Fr_knee_joint   (Knee Depan Kanan)
 9: FR_collar_joint                           9: Fl_knee_joint   (Knee Depan Kiri)
10: FR_hip_joint                             10: Br_knee_joint   (Knee Belakang Kanan)
11: FR_knee_joint                            11: Bl_knee_joint   (Knee Belakang Kiri)
```

### 🔬 Bukti Statistik Normalizer `policy.pt`
Dari ekstraksi buffer `obs_normalizer` pada `policy.pt`:
- **Indeks [6 : 9] (Gravity)**: Mean $= [0.0039, -0.0009, \mathbf{-0.9989}]$ $\rightarrow$ Robot tegak adalah $[0, 0, -1]$.
- **Indeks [12 : 15] (4 Roll)**: $\sigma \approx 0.13\text{ rad}$ (seragam 4 roll).
- **Indeks [16 : 19] (4 Hip)**: $\sigma \approx 0.16\text{ rad}$ (seragam 4 hip).
- **Indeks [20 : 23] (4 Knee)**: $\sigma \approx 0.12\text{ rad}$ (seragam 4 knee).

### 🛡️ Proteksi 2-Lapis Remapping di Controller:
1. **Lapis 1 - String Name Lookup**: Mengaitkan nama joint pesan ROS (`FR_collar_joint` / `Fr_roll_joint`) langsung ke indeks tensor Isaac `[0..11]`.
2. **Lapis 2 - Matriks Permutasi Indeks (Fallback)**:
   ```python
   # Dari CAN Hardware (BL, BR, FL, FR) -> Urutan Isaac (Roll, Hip, Knee):
   ROS_TO_ISAAC = [9, 6, 3, 0, 10, 7, 4, 1, 11, 8, 5, 2]

   # Dari output Policy Isaac -> Dikembalikan ke motor CAN Hardware:
   ISAAC_TO_ROS = [3, 7, 11, 2, 6, 10, 1, 5, 9, 0, 4, 8]
   ```

---

## 4. Tabel Pemetaan & Urutan Joint (Isaac Lab vs ROS Hardware)

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

---

## 5. Parameter Aktuator & Gain Kontrol (RobStride RS00)

- **Frekuensi Kontrol Policy**: `50 Hz` ($\Delta t = 0.02\text{ s}$)
- **Frekuensi Loop CAN Hardware**: `200 Hz`
- **Action Scale**: `0.25` ($q_{\text{des}} = q_0 + 0.25 \times a_{\text{policy}}$)
- **Stiffness Gain ($K_p$)**: `25.0`
- **Damping Gain ($K_d$)**: `1.0`

---

## 6. Konfigurasi & Panduan Joystick (Hardware & Virtual)

### 🎮 1. Joystick Fisik yang Didukung
Controller membaca device standar `/dev/input/js0` melalui package `joy`:
- **PlayStation 4 / DualShock 4** (USB / Bluetooth)
- **Xbox 360 / Xbox One / Series** (USB / Bluetooth)
- **Logitech F710 / Gamesir / General HID Gamepad**

### 🕹️ 2. Pemetaan Tombol & Stik Analog (Mapping)

#### 🔘 Tombol Perintah (State Machine):
| Tombol PS4 | Tombol Xbox | Indeks ROS (`msg.buttons`) | Aksi Robot |
| :---: | :---: | :---: | :--- |
| **`X` (Cross)** | **`A`** | `Button 0` | **`STANDUP`**: Robot perlahan berdiri ke sudut nominal $q_0$ (Roll: $0.0$, Hip: $-1.5$, Knee: $+1.5\text{ rad}$) selama 2 detik. |
| **`Lingkaran` (Circle)** | **`B`** | `Button 1` | **`WALK`**: Mengaktifkan model Policy RL PPO (50 Hz). |
| **`Kotak` (Square)** | **`X`** | `Button 2` | **`E-STOP / STANDBY`**: Menghentikan gerakan policy seketika dan duduk. |

#### 🕹️ Stik Analog (Pergerakan Robot):
| Input Analog | Axis Index | Rentang Nilai | Perintah Gerak |
| :--- | :---: | :---: | :--- |
| **Stik Kiri Vertikal** | `axes[1]` | $[-1.0, 1.0]$ | Kecepatan Maju / Mundur ($v_x$) |
| **Stik Kiri Horizontal** | `axes[0]` | $[-0.5, 0.5]$ | Kecepatan Geser Samping / Strafe ($v_y$) |
| **Stik Kanan Horizontal** | `axes[3]` / `axes[2]` | $[-1.2, 1.2]$ | Kecepatan Putar Badan / Yaw ($\omega_z$) |

---

### 💻 3. Alternatif: Virtual Joystick (Jika Tanpa Gamepad Fisik)

#### Opsi A: Virtual Gamepad CLI (Headless)
Jalankan script virtual controller di terminal:
```bash
python3 /home/erc/virtual_gamepad_headless.py
```
*Perintah interaktif:*
- Ketik `a` + Enter $\rightarrow$ Stand Up
- Ketik `b` + Enter $\rightarrow$ Walk (RL Active)
- Ketik `x` + Enter $\rightarrow$ Emergency Standby
- Ketik `ly 0.5` + Enter $\rightarrow$ Maju $0.5\text{ m/s}$

#### Opsi B: Publish Manual via ROS 2 `/cmd_vel`
```bash
# Perintah maju 0.4 m/s:
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.4, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 20
```

---

## 7. SOP Prosedur Deployment Hardware (Langkah Demi Langkah)

> [!IMPORTANT]
> **Tahap Persiapan Keselamatan (Wajib Gantung Robot)**:
> 1. Gantung robot pada tali pengaman (*rig / gantry*) hingga ke-4 kaki menggantung bebas di udara.
> 2. Pastikan baterai 24V / 6S terpasang aman dan saklar utama motor siap dinyalakan.

---

### 🚀 Urutan Terminal Eksekusi:

#### 🔹 Terminal 1: Inisialisasi Bus CAN Motor (can0 & can1)
```bash
cd /home/erc/sim2real
sudo ./scripts/bringup_canbus.sh
```

#### 🔹 Terminal 2: Jalankan Driver IMU (/dev/ttyUSB0)
```bash
source /opt/ros/humble/setup.bash
source /home/erc/nxp_jaguar/install/setup.bash
ros2 run serial_imu talker
```

#### 🔹 Terminal 3: Jalankan Driver CAN Motor RS00 (200 Hz)
```bash
cd /home/erc/sim2real
source /opt/ros/humble/setup.bash
python3 scripts/can_hardware_node.py
```
*(Node ini menginisialisasi 12 motor RS00, menerapkan sudut offset kalibrasi nol, dan melakukan soft ramp ke posisi STANDBY).*

#### 🔹 Terminal 4: Buka Live Dashboard Diagnostik Sensor & State
```bash
cd /home/erc/sim2real
source /opt/ros/humble/setup.bash
python3 scripts/check_states.py
```
*(Verifikasi manual: Gerakkan kaki robot satu per satu di udara dan pastikan baris sudut pada tabel bergerak sesuai nama sendi yang digerakkan)*.

#### 🔹 Terminal 5: Jalankan Controller Utama RL Sim-to-Real
```bash
cd /home/erc/sim2real
source /opt/ros/humble/setup.bash
python3 scripts/nxp_jaguar_controller.py
```

---

## 8. Fitur Keselamatan & E-Stop

1. **Auto E-Stop Kemiringan (*Tilt Protection*)**:
   - Jika kemiringan robot melebihi **$60^\circ$** ($g_z > -0.5$), controller secara otomatis memutus gerak policy dan kembali ke `STANDBY`.
2. **Proteksi Hilang Koneksi Sensor (*Sensor Timeout Guard*)**:
   - Jika sinyal IMU atau CAN terputus, controller otomatis menghentikan aksi.
3. **Pemeriksaan Overheat**:
   - Jika suhu motor melebihi **$75^\circ\text{C}$**, sistem akan mencetak peringatan darurat.

---

## 9. Validasi & Visualisasi MuJoCo Sim-to-Sim

Framework ini menyertakan simulator independen berbasis **MuJoCo** untuk memverifikasi model kontrol RL sebelum dijalankan di hardware fisik.

### 🎮 Fitur Sim-to-Sim MuJoCo:
- **Model 3D NXP Jaguar Autentik**: Menggunakan geometri CAD STL asli (`Base_body.STL`, `*_coxa_roll.STL`, `*_hip_pitch.STL`, `*_tibia_pitch.STL`).
- **4 Pilihan Medan (*Terrain*)**: `flat` (datar), `rough` (bergelombang natural), `stairs` (tangga piramida), dan `obstacles` (batu pijakan).
- **Auto-Follow Camera**: Kamera 3D viewer otomatis mengikuti robot ke mana pun berjalan.
- **Kontrol Keyboard Non-Blocking & Gamepad**:
  - `1` $\rightarrow$ Standby / Folded Pose
  - `2` $\rightarrow$ Minimum-Jerk Standup Trajectory (2.0 detik)
  - `3` $\rightarrow$ Walk Mode (RL Policy Active)
  - `W` / `S` $\rightarrow$ Maju / Mundur ($v_x$)
  - `A` / `D` $\rightarrow$ Geser Kiri / Kanan ($v_y$)
  - `Q` / `E` $\rightarrow$ Putar Kiri / Kanan ($\omega_z$)
  - `SPACE` $\rightarrow$ Stop Kecepatan ($v_x=0, v_y=0, \omega_z=0$)

### 🚀 Cara Menjalankan:
```bash
# Opsi 1: Direct Python Runner
python3 sim2sim/sim2sim_mujoco.py --terrain flat
python3 sim2sim/sim2sim_mujoco.py --terrain rough

# Opsi 2: Menggunakan ROS 2 Launch
ros2 launch jaguar_control sim2sim_mujoco.launch.py terrain:=flat
```
