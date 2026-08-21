# 🐾 NXP Jaguar Quadruped: Sim-to-Real Deployment (Branch: `cpp`)
**Reinforcement Learning Control: Isaac Lab 3.0 (DreamWaQ) ➡️ RobStride RS00 via ROS 2 (C++ Hard Real-Time Driver)**

---

## 📌 Ringkasan Branch `cpp`
Branch ini mengimplementasikan **Hard Real-Time C++ CAN Driver (`robstride_can_node`)** berbasis Linux SocketCAN native dengan prioritas thread real-time (**`SCHED_FIFO`, Priority 80**). Dirancang khusus untuk meminimalkan latensi komunikasi (< 1 ms), menghilangkan *jitter* timer Python GIL, dan menjamin frekuensi loop deterministik **200 Hz** untuk pengendalian 12 motor RobStride RS00 secara simultan.

---

## 📑 Daftar Isi
1. [Arsitektur Sistem (High-Level ke Low-Level C++)](#1-arsitektur-sistem-high-level-ke-low-level-c)
2. [Rincian Komponen Low-Level C++ Driver](#2-rincian-komponen-low-level-c-driver)
3. [Spesifikasi Observasi Policy RL (48-Dimensi)](#3-spesifikasi-observasi-policy-rl-48-dimensi)
4. [Mekanisme Remapping Joint (Isaac Lab vs ROS Hardware)](#4-mekanisme-remapping-joint-isaac-lab-vs-ros-hardware)
5. [Tabel Pemetaan Joint, ID Motor, dan CAN Bus](#5-tabel-pemetaan-joint-id-motor-dan-can-bus)
6. [Gain Kontrol & Parameter Aktuator RobStride](#6-gain-kontrol--parameter-aktuator-robstride)
7. [Panduan Teleoperasi Joystick (Hardware & Virtual)](#7-panduan-teleoperasi-joystick-hardware--virtual)
8. [🚀 SOP Urutan Eksekusi Menjalankan Robot (Langkah Demi Langkah)](#8--sop-urutan-eksekusi-menjalankan-robot-langkah-demi-langkah)
9. [Tool Diagnostik & Kalibrasi Hardware](#9-tool-diagnostik--kalibrasi-hardware)
10. [Fitur Keselamatan & E-Stop](#10-fitur-keselamatan--e-stop)
11. [Validasi Sim-to-Sim MuJoCo](#11-validasi-sim-to-sim-mujoco)

---

## 1. Arsitektur Sistem (High-Level ke Low-Level C++)

```
[ HIGH-LEVEL: RL Policy ]
      │  Model: TorchScript JIT (`policy.pt`) dilatih di Isaac Lab 3.0 (DreamWaQ)
      │  Frekuensi: 50 Hz (dt = 0.02 s) | Input: 48-Dim Observasi | Output: 12-Dim Target Δq
      ▼
[ MID-LEVEL: ROS 2 Controller Node (`scripts/nxp_jaguar_controller.py`) ]
      │  • Membaca IMU (`/Imu_data`), Joy/Teleop (`/joy`, `/cmd_vel`), dan Joint States (`/robot_joint_states`)
      │  • Finite State Machine: STANDBY ──(Btn A)──> STANDUP ──(Btn B)──> WALK ──(Btn X)──> E-STOP
      │  • Remapping: Isaac Order (Roll->Hip->Knee) ⇄ ROS Hardware Order (BL->BR->FL->FR)
      │  • Menghitung Desired Angle: q_des = q_nominal + 0.25 * action
      │  • Publish ke: `/joint_command` (std_msgs/Float64MultiArray)
      ▼
[ LOW-LEVEL: C++ Hard Real-Time CAN Node (`src/robstride_can_node.cpp`) ]
      │  • Multithreaded Engine: Dual SocketCAN Threads (`can0` & `can1`) via `robstride_can_bus.hpp`
      │  • Real-Time Scheduler: Linux `SCHED_FIFO` (Priority 80)
      │  • Protocol: RobStride RS00 Frame Bit-Packing (`robstride_protocol.hpp`)
      │  • Deterministik: Loop Timer 200 Hz (dt = 0.005 s) tanpa overhead Python GIL
      │  • Publish state aktual ke: `/robot_joint_states` (sensor_msgs/JointState)
      ▼
[ HARDWARE: 12x RobStride RS00 Motor & Hiwonder 9-DOF IMU ]
```

---

## 2. Rincian Komponen Low-Level C++ Driver

Modul C++ berlokasi di [`include/jaguar_control/`](file:///home/erc/sim2real/include/jaguar_control/) dan [`src/`](file:///home/erc/sim2real/src/):

| File | Kelas / Fungsi Utama | Deskripsi Teknis |
| :--- | :--- | :--- |
| **`robstride_protocol.hpp`** | `RobStrideProtocol` | Konversi float $\leftrightarrow$ uint16 bit-packing (Posisi, Kecepatan, $K_p$, $K_d$, Torsi feedforward), enkapsulasi frame CAN 29-bit extended ID. |
| **`robstride_can_bus.hpp`** | `RobStrideCANBus` | Abstraksi native Linux POSIX SocketCAN (`AF_CAN`, `SOCK_RAW`), pengiriman non-blocking dan pembacaan multithreaded dengan filter CAN ID. |
| **`robstride_hardware_manager.hpp`** | `RobStrideHardwareManager` | Mengatur dual bus CAN paralel (`can0` untuk Belakang, `can1` untuk Depan), soft-ramp trajectory generator saat startup, interpolasi target sudut, dan kompensasi offset nol mekanikal. |
| **`robstride_can_node.cpp`** | `RobStrideCANNode` (ROS 2) | Node utama ROS 2: thread deterministik 200 Hz, subscriber `/joint_command`, publisher `/robot_joint_states`, dan handler real-time priority `SCHED_FIFO`. |

---

## 3. Spesifikasi Observasi Policy RL (48-Dimensi)

Model neural network (*actor policy*) menerima vektor observasi berukuran tepat **48 dimensi**:

| Rentang Indeks | Nama State | Dimensi | Satuan | Deskripsi & Nilai Nominal |
| :---: | :--- | :---: | :---: | :--- |
| `[0 : 3]` | **`base_lin_vel`** | 3 | $\text{m/s}$ | Kecepatan linier badan robot dalam *body frame* $[v_x, v_y, v_z]$ |
| `[3 : 6]` | **`base_ang_vel`** | 3 | $\text{rad/s}$ | Kecepatan sudut gyro dalam *body frame* $[\omega_x, \omega_y, \omega_z]$ |
| `[6 : 9]` | **`projected_gravity`** | 3 | unit | Vektor gravitasi proyeksi $[g_x, g_y, g_z]$ ($[0, 0, -1]$ saat tegak) |
| `[9 : 12]` | **`velocity_commands`** | 3 | $\text{m/s, rad/s}$ | Perintah joystick user $[v_x^{\text{cmd}}, v_y^{\text{cmd}}, \omega_z^{\text{cmd}}]$ |
| `[12 : 24]` | **`joint_pos_rel`** | 12 | $\text{rad}$ | Posisi sudut sendi relatif terhadap nominal: $(q_i - q_{0, i})$ |
| `[24 : 36]` | **`joint_vel`** | 12 | $\text{rad/s}$ | Kecepatan sudut 12 sendi motor $\dot{q}_i$ |
| `[36 : 48]` | **`actions`** | 12 | $\text{rad}$ | Aksi policy pada langkah sebelumnya $a_{t-1}$ (*last action*) |

$$\text{Total Dimensi Observasi} = 3 + 3 + 3 + 3 + 12 + 12 + 12 = \mathbf{48}$$

---

## 4. Mekanisme Remapping Joint (Isaac Lab vs ROS Hardware)

* **Driver Motor CAN Hardware**: Dikelompokkan **per kaki** (`BL`, `BR`, `FL`, `FR`).
* **Policy Isaac Lab**: Dikelompokkan **per tipe sendi** (semua `Roll`, semua `Hip`, semua `Knee`).

```
[Driver Motor CAN Hardware (C++ / ROS)]      [Model RL Policy Isaac Lab 3.0]
Dikelompokkan per KAKI (BL, BR, FL, FR)      Dikelompokkan per TIPE SENDI (Rolls, Hips, Knees)
───────────────────────────────────────      ────────────────────────────────────────────────
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

Matriks permutasi yang diterapkan pada `scripts/nxp_jaguar_controller.py`:
```python
ROS_TO_ISAAC = [9, 6, 3, 0, 10, 7, 4, 1, 11, 8, 5, 2]
ISAAC_TO_ROS = [3, 7, 11, 2, 6, 10, 1, 5, 9, 0, 4, 8]
```

---

## 5. Tabel Pemetaan Joint, ID Motor, dan CAN Bus

| Indeks Isaac | Nama Joint (Isaac Lab) | Sudut Standby ($q_0$) | Indeks ROS | Nama Joint di ROS (`parameters.py`) | ID Motor | CAN Bus |
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

## 6. Gain Kontrol & Parameter Aktuator RobStride

- **Frekuensi Policy Controller**: `50 Hz` ($\Delta t = 0.02\text{ s}$)
- **Frekuensi C++ Hardware Node**: `200 Hz` ($\Delta t = 0.005\text{ s}$)
- **Real-Time Priority**: `SCHED_FIFO` Priority `80`
- **Action Scale**: `0.25` ($q_{\text{des}} = q_0 + 0.25 \times a_{\text{policy}}$)
- **Stiffness ($K_p$)**: `25.0`
- **Damping ($K_d$)**: `1.0`

---

## 7. Panduan Teleoperasi Joystick (Hardware & Virtual)

Controller membaca gamepad standar (`/dev/input/js0`) via package ROS 2 `joy`.

### 🔘 Tombol Perintah (State Machine):
| Tombol PS4 | Tombol Xbox | Indeks ROS | Aksi State Robot |
| :---: | :---: | :---: | :--- |
| **`X` (Cross)** | **`A`** | `Button 0` | **`STANDUP`**: Transisi halus berdiri ke posisi nominal $q_0$ selama 2 detik. |
| **`Lingkaran`** | **`B`** | `Button 1` | **`WALK`**: Mengaktifkan model Policy RL PPO (50 Hz). |
| **`Kotak`** | **`X`** | `Button 2` | **`E-STOP / STANDBY`**: Menghentikan policy seketika dan duduk. |

### 🕹️ Stik Analog (Pergerakan Robot):
| Input Analog | Axis Index | Rentang | Perintah Gerak |
| :--- | :---: | :---: | :--- |
| **Stik Kiri Vertikal** | `axes[1]` | $[-1.0, 1.0]$ | Kecepatan Maju / Mundur ($v_x$) |
| **Stik Kiri Horizontal** | `axes[0]` | $[-0.5, 0.5]$ | Kecepatan Geser Samping / Strafe ($v_y$) |
| **Stik Kanan Horizontal** | `axes[3]` / `axes[2]` | $[-1.2, 1.2]$ | Kecepatan Putar Badan / Yaw ($\omega_z$) |

---

## 8. 🚀 SOP Urutan Eksekusi Menjalankan Robot (Langkah Demi Langkah)

> [!IMPORTANT]
> **Prosedur Keselamatan Wajib**:
> 1. Gantung robot pada rig pengaman (*gantry*) hingga ke-4 kaki menggantung bebas di udara sebelum pengujian.
> 2. Pastikan baterai 24V terhubung dan tombol emergency siap ditekan kapan saja.

---

### 🔨 Langkah Awal: Build Node C++ (Wajib Dilakukan Sekali)
```bash
cd /home/erc/sim2real
colcon build --packages-select jaguar_control --symlink-install
source install/setup.bash
```

---

### OPSI A: Eksekusi Multi-Terminal (Sangat Direkomendasikan untuk Debugging & Observasi)

Buka 5 tab terminal terpisah dan jalankan perintah secara berurutan:

#### 🔹 Terminal 1: Inisialisasi Bus CAN (1 Mbps)
```bash
cd /home/erc/sim2real
sudo ./scripts/bringup_canbus.sh
```
*Verifikasi: Pastikan `can0` dan `can1` aktif dengan bitrate 1000000.*

#### 🔹 Terminal 2: Jalankan Driver IMU
```bash
cd /home/erc/sim2real
./scripts/bringup_imu.sh
```
*Verifikasi: Muncul pesan publishing IMU di topik `/Imu_data`.*

#### 🔹 Terminal 3: Jalankan Driver C++ Hard Real-Time Node (200 Hz)
```bash
cd /home/erc/sim2real
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run jaguar_control robstride_can_node
```
*Output: C++ driver mengaktifkan thread SCHED_FIFO, mendeteksi 12 motor RS00, dan siap menerima target joint.*

#### 🔹 Terminal 4: Buka Live Dashboard Diagnostik Sensor
```bash
cd /home/erc/sim2real
source /opt/ros/humble/setup.bash
python3 scripts/check_states.py
```
*Verifikasi: Tabel live menampilkan 12 sudut sendi secara real-time dan orientasi IMU.*

#### 🔹 Terminal 5: Jalankan Controller Utama Sim-to-Real RL
```bash
cd /home/erc/sim2real
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run jaguar_control nxp_jaguar_controller.py
```

#### 🎮 Menggerakkan Robot:
1. Tekan tombol **`A` / `X` (Cross)** pada gamepad $\rightarrow$ Robot akan berdiri secara mulus (*Standup*).
2. Tekan tombol **`B` / `Circle`** pada gamepad $\rightarrow$ Robot masuk mode *Walk* (Policy RL aktif).
3. Gerakkan **Stik Analog Kiri** maju/mundur untuk mengontrol kecepatan.
4. Tekan tombol **`X` / `Square`** kapan saja untuk *Emergency Stop / Duduk*.

---

### OPSI B: One-Click Launch (ROS 2 Launch Full System)

Menjalankan seluruh pipeline (IMU, Joy, C++ CAN Node 200 Hz, dan RL Controller) dalam 1 perintah:
```bash
# 1. Pastikan CAN bus sudah aktif:
sudo /home/erc/sim2real/scripts/bringup_canbus.sh

# 2. Jalankan launch file (secara default menggunakan C++ hardware driver):
source /opt/ros/humble/setup.bash
source /home/erc/nxp_jaguar/install/setup.bash
source /home/erc/sim2real/install/setup.bash
ros2 launch jaguar_control sim2real.launch.py
```

---

## 9. Tool Diagnostik & Kalibrasi Hardware

Semua tool utilitas pendukung tersedia di folder `scripts/`:

* **Scan Motor ID di CAN Bus:**
  ```bash
  python3 scripts/scan_robostride_ids.py
  ```
* **Kalibrasi Posisi Nol Mekanikal (Zero Calibration):**
  *(Jalankan saat posisi kaki terlipat/duduk sempurna)*
  ```bash
  python3 scripts/set_robostride_zero.py
  ```
* **Test Transisi Duduk-Berdiri Mandiri (Tanpa RL):**
  ```bash
  python3 scripts/test_sit_stand.py
  ```
* **Inspeksi Sudut Sendi Pasif:**
  ```bash
  python3 scripts/check_joints.py
  ```

---

## 10. Fitur Keselamatan & E-Stop

1. **Auto E-Stop Kemiringan (*Tilt Protection*)**: Jika kemiringan robot melebihi **$60^\circ$** ($g_z > -0.5$), controller otomatis memutus aksi policy dan robot kembali duduk.
2. **Sensor Timeout Watchdog**: Jika sinyal IMU atau CAN terputus lebih dari 0.1 detik, sistem langsung menghentikan motor.
3. **Thermal Guard**: Peringatan darurat jika suhu motor melebihi **$75^\circ\text{C}$**.

---

## 11. Validasi Sim-to-Sim MuJoCo

Sebelum deploy ke robot fisik, validasi model RL dapat dilakukan di simulator MuJoCo:
```bash
python3 sim2sim/sim2sim_mujoco.py --terrain flat
python3 sim2sim/sim2sim_mujoco.py --terrain rough
```
*Kontrol keyboard di MuJoCo:*
* `1`: Standby (Folded)
* `2`: Stand Up (Minimum-jerk trajectory)
* `3`: Walk (RL Policy aktif)
* `W/S/A/D/Q/E`: Pergerakan badan robot
