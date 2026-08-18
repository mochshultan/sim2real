# 🐾 NXP Jaguar Sim-to-Sim Validation Framework (MuJoCo & Isaac Lab)
**Cross-Physics Engine Sim-to-Sim Verification & Teleoperation Pipeline for NXP Jaguar Quadruped Robot**

Repository ini merupakan framework **Sim-to-Sim** mandiri dan komprehensif untuk robot quadruped **NXP Jaguar**. Framework ini dirancang untuk memvalidasi model neural network RL (DreamWaQ / RSL-RL) secara independen di mesin fisika **MuJoCo** dan **Isaac Lab** sebelum dideploy ke robot fisik nyata (**Sim-to-Real**).

---

## 🏗️ Arsitektur Sistem Sim-to-Sim

```
                          ┌──────────────────────────────┐
                          │   Trained JIT Actor Policy   │
                          │        (policy.pt)           │
                          └──────────────┬───────────────┘
                                         │ Action (12D)
                                         ▼
┌─────────────────────────┐   ┌──────────────────────────┐   ┌──────────────────────────┐
│   Teleop / Gamepad      │   │  50 Hz Control Loop      │   │     MuJoCo Physics       │
│  • Keyboard (Terminal)  ├──►│  • ObservationBuilder    ├──►│  • 500 Hz Decimation     │
│  • USB/BT Joystick      │   │  • Minimum-Jerk Trajectory│  │  • Pure CAD Mesh Contact │
└─────────────────────────┘   └──────────▲───────────────┘   │  • Camera Auto-Follow    │
                                         │ Sensors           └──────────────────────────┘
                                         └───────────────────────────────┘
```

---

## 🌟 Fitur Utama

1. **Model CAD 3D & Kinematika Asli NXP Jaguar (100% Autentik)**:
   - Menggunakan mesh STL asli (`Base_body.STL`, `*_coxa_roll.STL`, `*_hip_pitch.STL`, `*_tibia_pitch.STL`).
   - Struktur inersia ($5.2\text{ kg}$ base), geometri, panjang link, dan batas torsi motor RobStride RS00 ($17\text{ N}\cdot\text{m}$).
   - **Kontak Fisika Mesh Murni**: Menggunakan geometri CAD asli telapak kaki tanpa bola bantalan buatan yang mendistorsi visual.
2. **Auto-Load JIT Policy Terbaru**:
   - Otomatis mendeteksi dan me-load model TorchScript JIT (`policy.pt`) paling mutakhir dari direktori log training Isaac Lab (`~/IsaacLab/logs/rsl_rl/nxp_jaguar_rough/`).
3. **Trajektori Berdiri Halus (*Minimum-Jerk Standup*)**:
   - Transisi dari posisi tidur di lantai (seluruh sendi $= 0.0\text{ rad}$) menuju sudut berdiri nominal $q_0 = \pm 1.50\text{ rad}$ menggunakan trajektori polinomial berderajat lima $S(\alpha) = 10\alpha^3 - 15\alpha^4 + 6\alpha^5$ selama $2.0\text{ detik}$ bebas oleng.
4. **Camera Auto-Follow Dinamis**:
   - Kamera viewer MuJoCo secara otomatis dan mulus mengikuti pergerakan robot (*3rd-person isometric view*) kemanapun robot berjalan.
5. **Terminal Non-Blocking Keyboard & Gamepad Support**:
   - Input keyboard ditangkap langsung secara real-time dari terminal tanpa perlu menekan Enter atau mengklik window simulasi.

---

## 📊 Spesifikasi Tensor Observasi (48-D Vector)

Tensor observasi yang dikirimkan ke model JIT **100% identik dan presisi** dengan konfigurasi training Isaac Lab (`LocomotionVelocityRoughEnvCfg.observations.policy`):

| No | Komponen Observasi | Dimensi | Formula Matematis | Keterangan |
|---|---|---|---|---|
| 1 | **`base_lin_vel`** | 3D | $R^T \mathbf{v}_{\text{world}}$ | Kecepatan linear pada frame bodi ($v_x, v_y, v_z$) |
| 2 | **`base_ang_vel`** | 3D | $\boldsymbol{\omega}_{\text{body}}$ | Kecepatan sudut IMU Gyro pada frame bodi ($\omega_x, \omega_y, \omega_z$) |
| 3 | **`projected_gravity`** | 3D | $R^T [0, 0, -1]^T$ | Vektor proyeksi gravitasi bumi pada frame bodi |
| 4 | **`velocity_commands`** | 3D | $[v_x^{\text{cmd}}, v_y^{\text{cmd}}, \omega_z^{\text{cmd}}]$ | Perintah kecepatan teleoperasi dari user |
| 5 | **`joint_pos_rel`** | 12D | $q_{\text{isaac}} - q_0$ | Posisi sudut sendi relatif terhadap sudut nominal $q_0$ |
| 6 | **`joint_vel_rel`** | 12D | $\dot{q}_{\text{isaac}}$ | Kecepatan sudut masing-masing sendi |
| 7 | **`actions`** | 12D | $a_{t-1}$ | Aksi keluaran neural network pada step sebelumnya |
| **Total** | **Observation** | **48D** | Tensor float32 | Input langsung untuk Policy Actor |

---

## 🔄 Pemetaan Indeks Sendi (Joint Order Remapping)

Karena urutan sendi pada MuJoCo MJCF dan Isaac Lab berbeda, modul [`observation_builder.py`](file:///home/shultan/jaguar_sim2sim/observation_builder.py) melakukan konversi indeks secara otomatis:

- **Urutan Sendi Isaac Lab (Rolls $\to$ Hips $\to$ Knees)**:
  `[0: Fr_r, 1: Fl_r, 2: Br_r, 3: Bl_r, 4: Fr_h, 5: Fl_h, 6: Br_h, 7: Bl_h, 8: Fr_k, 9: Fl_k, 10: Br_k, 11: Bl_k]`
- **Urutan Sendi MuJoCo (Per-Kaki: FR $\to$ FL $\to$ BR $\to$ BL)**:
  `[0..2: Fr_r, Fr_h, Fr_k | 3..5: Fl_r, Fl_h, Fl_k | 6..8: Br_r, Br_h, Br_k | 9..11: Bl_r, Bl_h, Bl_k]`
- **Konversi Indeks**:
  - `MUJOCO_TO_ISAAC = [0, 3, 6, 9, 1, 4, 7, 10, 2, 5, 8, 11]`
  - `ISAAC_TO_MUJOCO = [0, 4, 8, 1, 5, 9, 2, 6, 10, 3, 7, 11]`

---

## ⚙️ Parameter Aktuator & Kontrol (RobStride RS00)

- **Nominal Standing Angles ($q_0$)**:
  - Roll Joints: `0.00 rad`
  - Hip Pitch Joints: `-1.50 rad`
  - Knee Joints: `+1.50 rad`
- **Action Scale**: `0.25` ($\text{Target } q = q_0 + 0.25 \times a_t$)
- **Gain PD Kontroler Motor**:
  - Mode Jalan (`WALK`): $K_p = 25.0\text{ N}\cdot\text{m/rad}, K_d = 1.0\text{ N}\cdot\text{m}\cdot\text{s/rad}$
  - Mode Berdiri (`STANDUP`): $K_p = 35.0\text{ N}\cdot\text{m/rad}, K_d = 2.0\text{ N}\cdot\text{m}\cdot\text{s/rad}$
  - Batas Torsi Maksimum: $\tau_{\max} = 17.0\text{ N}\cdot\text{m}$
- **Frekuensi Kontrol**:
  - Control Step (Policy Evaluation): **50 Hz** ($\Delta t = 0.02\text{ s}$)
  - Physics Simulation Step: **500 Hz** ($\Delta t = 0.002\text{ s}$, Decimation = 10)

---

## 🚀 Cara Menjalankan

### 1. MuJoCo Sim-to-Sim (Rekomendasi Utama)
Simulasi fisik independen yang sangat ringan, cepat, dan responsif. Mendukung 4 jenis medan (terrain):

```bash
conda activate isaaclab
cd ~/jaguar_sim2sim

# A. Medan Datar (Flat Plane)
python3 sim2sim_mujoco.py --terrain flat

# B. Medan Bergelombang Kasar (Rough Natural Bumps & Hills)
python3 sim2sim_mujoco.py --terrain rough

# C. Tangga Bertingkat (Stepped Pyramid Stairs)
python3 sim2sim_mujoco.py --terrain stairs

# D. Rintangan Bebatuan (Stepping Stone Obstacles)
python3 sim2sim_mujoco.py --terrain obstacles
```

*Jika ingin memuat checkpoint spesifik atau run tertentu:*
```bash
python3 sim2sim_mujoco.py --terrain rough --load_run 2026-08-17_14-07-27
# atau path absolut:
python3 sim2sim_mujoco.py --terrain rough --policy /path/to/exported/policy.pt
```

---

### 2. Isaac Lab Sim-to-Sim (Interactive Mode)
Simulasi berbasis Isaac Sim dengan visualisasi Viser/GUI:

```bash
conda activate isaaclab
cd ~/jaguar_sim2sim
python3 sim2sim_isaaclab.py --task flat --viz viser
```

---

## 🕹️ Panduan Kontrol Teleoperasi

### ⌨️ Kontrol Keyboard (Langsung Ketik di Terminal):
| Tombol | Aksi | Deskripsi |
| :---: | :--- | :--- |
| **`1`** | **`STANDBY`** | Robot duduk santai ke posisi 0 di lantai ($q = 0.0\text{ rad}$) |
| **`2`** | **`STANDUP`** | Robot berdiri tegak secara mulus (*Minimum-Jerk*) ke $q_0 = \pm 1.5\text{ rad}$ |
| **`3`** | **`WALK`** | Mengaktifkan neural network policy RL DreamWaQ (50 Hz) |
| **`W / S`** | **Maju / Mundur** | Mengatur kecepatan linear $v_x$ ($\pm 0.2\text{ m/s}$) |
| **`A / D`** | **Geser Kiri / Kanan** | Mengatur kecepatan lateral strafe $v_y$ ($\pm 0.15\text{ m/s}$) |
| **`Q / E`** | **Putar Kiri / Kanan** | Mengatur kecepatan sudut putar yaw $\omega_z$ ($\pm 0.3\text{ rad/s}$) |
| **`Space`** | **Stop / Diam** | Mereset semua perintah kecepatan menjadi $0.0$ |
| **`R`** | **Reset Robot** | Mereset simulasi robot kembali ke titik awal (Posisi 0) |

---

### 🎮 Kontrol Gamepad Joystick:
- **Tombol A / Cross (X)**: Transisi ke `STANDUP`
- **Tombol B / Circle (O)**: Transisi ke `WALK` (Jalan Aktif)
- **Tombol X / Square (Kotak)**: Kembali ke `STANDBY` (Duduk)
- **Stik Analog Kiri**: Dorong depan/belakang ($v_x$), geser samping ($v_y$)
- **Stik Analog Kanan**: Geser kiri/kanan untuk rotasi yaw ($\omega_z$)

---

## 📁 Struktur File & Direktori

```text
~/jaguar_sim2sim/
├── README.md                 # Dokumentasi komprehensif & panduan lengkap
├── sim2sim_mujoco.py         # Main Simulator MuJoCo (State machine, decimation, teleop, camera follow)
├── sim2sim_isaaclab.py       # Main Simulator Isaac Lab (Viser visualizer & teleop)
├── observation_builder.py    # Modul penyusun tensor 48-D & transformasi kinematika
└── models/
    ├── scene.xml             # XML Scene MuJoCo (lighting, floor texture, visualizer)
    ├── nxp_jaguar.xml        # Model MJCF NXP Jaguar (body, joints, pure CAD mesh, actuators)
    ├── urdf/
    │   └── nxp_jaguar.urdf   # File URDF asli NXP Jaguar
    └── meshes/               # 13 File CAD 3D STL asli NXP Jaguar
```

---

## 🏁 Kesiapan Sim-to-Real

Pipeline data pada framework ini dirancang secara **1-to-1 matching** dengan controller onboard robot fisik (C++ LibTorch / Python ROS 2):
1. **Sensors Ingestion**: IMU Quat $\to$ Body Lin/Ang Vel $\to$ Projected Gravity.
2. **Motor Commands**: Output JIT Action $[-1, 1] \times 0.25 + q_0 \to$ CAN Bus RobStride RS00.
3. **Safety State Machine**: Transisi `STANDBY` $\to$ `STANDUP` $\to$ `WALK` terbukti stabil dan bebas sentakan mekanis.
