# Model Jaguar dengan referensi 0 rad

Jalankan model 19999 melalui `sim2real_zero.launch.py`. Launcher ini memakai
stack ros2_control yang sama dengan launcher nominal: controller, pembatasan gerak, driver
CAN, teleop, dan safety tidak diduplikasi.

Referensi policy dan pose berdiri adalah dua konfigurasi berbeda:

| Bagian | Referensi 0 rad / model 19999 | Referensi nominal / model 29996 |
| --- | --- | --- |
| Launcher | `sim2real_zero.launch.py` | `sim2real.launch.py` |
| Posisi dalam observasi | `q` | `q - q0` |
| Target policy sebelum clamp | `0.25 * clip(action, -10, 10)` | `q0 + 0.25 * action` |
| Target selama ramp WALK | `stand + ramp * (target_policy - stand)` | Rumus yang sama |
| Pose STANDUP / STAND_HOLD | `STAND_ANGLE`, ditambah trim IMU jika aktif | Pose yang sama |

`q` adalah posisi sendi setelah mapping dan kalibrasi encoder. Referensi 0 rad
tidak mengubah mechanical zero dan tidak membuat robot berdiri dengan semua
sendi pada 0 rad. Default controller tetap STANDBY pasif saat launch.

## Build dan menjalankan

Hentikan launcher sebelumnya, lalu build setelah perubahan kode atau penambahan
asset model:

```bash
cd /home/qr1/sim2real
source scripts/build.sh
```

Setelah build berhasil:

```bash
sudo ./scripts/bringup_canbus.sh
ros2 launch jaguar_control sim2real_zero.launch.py
```

Untuk terminal baru, muat environment dengan
`source scripts/source_ros.sh` dan `source install/setup.bash` dari root repo.
Log controller harus memuat `Policy joint reference: zero` dan path policy
`policy_adaboot_l2c2_zero_reference_clearance5cm_19999_20261111.pt`.
Xbox: **A** untuk berdiri, **Start/Menu** untuk masuk WALK; Start lagi untuk
kembali ke STAND_HOLD. Model ini memiliki validasi offline; file manifest tidak
menyatakan bahwa model sudah diuji pada hardware.

Pose dasar berdiri khusus zero-reference: coxa `0.0`, semua hip `-1.25`,
dan semua knee `+1.25` rad (`ZERO_STAND_JOINT_POS` di controller). Pose ini
dipakai saat STANDUP, STAND_HOLD, kembali dari WALK, dan awal walk ramp.
Auto-level IMU yang aktif tetap dapat menambahkan trim terbatas.

## Log otomatis per sesi

Saat `sim2real_zero.launch.py` dijalankan, controller otomatis merekam ke
`~/.ros/jaguar_control/sessions/sim2real_zero_<waktu-UTC>_<id>.jsonl`.
Path lengkap dicetak sebagai `Session recording: ...` pada startup. Setiap
startup membuat file baru; file tetap tersimpan setelah launcher dihentikan.
Perekaman dimulai di STANDBY, termasuk ketika menunggu sensor, dan berlanjut
pada STANDUP, STAND_HOLD, WALK, SITDOWN, SAFE_PARK, dan DISABLED.

Format JSONL berisi satu objek JSON per baris:

- `metadata`: path model, referensi, pose berdiri, skala observasi, serta
  `joint_names` yang menentukan urutan semua array 12 motor (FR/FL/BR/BL,
  masing-masing kelompok roll, hip, lalu knee).
- `sample`: waktu wall/monotonic/ROS, `state_start`, `state`, status hardware,
  validitas dan umur sensor, feedback posisi (rad), kecepatan (rad/s), torsi
  (Nm), quaternion, gyro dan command velocity. Satu sample per siklus 50 Hz.
- `observation`: 45 nilai dengan layout dan skala pada metadata. Saat WALK,
  ini persis observasi yang diberikan ke policy sebelum pembaruan last action;
  `policy_history` menyimpan seluruh input 5 × 45.
- `policy_action`: 12 output policy sebelum EMA dan skala 0.25. Model 19999
  sudah melakukan clipping aksi di dalam TorchScript. Di luar WALK nilainya
  `null`, `policy_history` juga `null`, dan `policy_evaluated` bernilai false.
- `motor_output`: 12 target posisi dan kecepatan serta Kp/Kd yang benar-benar
  dipublikasikan setelah ramp dan clamp batas posisi. Nilainya `null` jika
  tidak ada command pada siklus itu. Kp/Kd bukan perintah torsi; torsi terukur
  tersedia di `feedback_torque_nm`.
- `estop`: alasan stop dan command pasif yang dipublikasikan, termasuk stop
  yang datang dari callback di luar siklus kontrol.
- `summary`: jumlah sample yang terbuang akibat antrean penuh.

Observasi ketika sensor belum valid adalah snapshot nilai controller, bukan
input inference; periksa `imu_valid`, `joints_valid`, umur sensor, dan
`policy_evaluated` sebelum menganalisisnya.

Penulisan file memakai thread terpisah dengan antrean terbatas. Antrean penuh
membuang sample dan mencetak peringatan; error penulisan juga dilaporkan.
Buffer di-flush setiap sekitar satu detik dan antrean dikuras saat shutdown
normal. Penghentian paksa atau listrik padam dapat menghilangkan bagian akhir
log. Tidak ada rotasi atau penghapusan log otomatis.

Untuk mengubah direktori atau menonaktifkan rekaman, atur parameter ROS pada
section `nxp_jaguar_controller.ros__parameters` dalam salinan konfigurasi penuh,
lalu gunakan `config_file:=/path/absolut/custom.yaml`:

```yaml
record_session: true
session_log_dir: /home/qr1/sim2real/log/zero_sessions
```

`record_session` default true untuk reference `zero`, false untuk `nominal`.
Parameter ini dan `session_log_dir` dibaca saat startup, bukan flag launch
langsung. Restart launcher setelah mengubahnya.

## Flag launch

Format flag adalah `nama:=nilai`. Semua flag di bawah dibaca ketika launch.
Daftar aktual dapat diperiksa tanpa menjalankan node:

```bash
ros2 launch jaguar_control sim2real_zero.launch.py --show-args
```

| Flag | Default pada launcher zero | Fungsi |
| --- | --- | --- |
| `policy_path` | `models/policy_adaboot_l2c2_zero_reference_clearance5cm_19999_20261111.pt` | Ganti actor TorchScript dengan model yang dilatih untuk referensi zero dan kontrak input/output yang sama. Path default di-resolve ke path absolut. |
| `history_initialization` | `repeat` | Ulangi observasi awal pada kelima frame, dengan feedback aksi awal nol. CLI override tetap berlaku; `zeros` tersedia untuk kompatibilitas policy lain. |
| `policy_reference` | `zero`, hanya menerima `zero` | Menjaga pasangan model dan referensi; untuk nominal gunakan launcher nominal. |
| `config_file` | `config/sim2real.yaml` | File parameter ROS untuk controller, teleop, mux, joy opsional, dan driver CAN. |
| `domain_id` | `$ROS_DOMAIN_ID`, atau `1` jika belum diset | Domain DDS; terminal dan mesin lain harus menggunakan nilai yang sama. |
| `with_imu` | `true` | Jalankan driver serial IMU. |
| `with_teleop` | `true` | Jalankan sumber Xbox. Keyboard dijalankan terpisah. |
| `with_joy` | `false` | Jalankan `joy_node` dengan output `/joy_raw`; biasanya pembaca Xbox langsung sudah cukup. |
| `with_hardware` | `true` | Jalankan driver CAN. Nilai `false` hanya menonaktifkan driver ini, bukan seluruh stack. |
| `mock_hardware` | `false` | Gunakan hardware mock tanpa membuka CAN; bukan simulasi dinamika. |
| `with_controller` | `true` | Jalankan controller RL. |
| `startup_clear_faults` | `true` | C++ driver: clear fault sekali sebelum enable saat startup. |

Nilai CLI mengalahkan default launcher. Controller membaca file YAML, kemudian
menerapkan override `policy_path`, `policy_reference`, dan `history_initialization`. Pada launcher nominal, default reference
adalah `nominal` dan default model adalah `models/policy.pt`.

## Parameter melalui YAML

Parameter berikut **bukan flag launch langsung**. Edit file YAML atau salin
seluruh `config/sim2real.yaml` menjadi file konfigurasi sendiri dan pilih lewat
`config_file:=/path/absolut/custom.yaml`. Pertahankan nama node serta struktur
`ros__parameters`. File `config_file` menggantikan pilihan file default; jangan
mengandalkannya sebagai patch parsial atas file default.

Nilai di bawah adalah konfigurasi repo saat ini. Controller dan driver menyimpan
banyak parameter dalam atribut saat startup, jadi restart launch setelah
mengubah konfigurasi. `ros2 param set` tidak menjamin perilaku berjalan berubah.

| Bagian YAML / node | Parameter | Default | Fungsi / satuan |
| --- | --- | --- | --- |
| `/**` | `max_vx`, `max_vy` | `1.0`, `1.0` | Batas kecepatan per sumbu X/Y, m/s. Dipakai bersama oleh controller, Xbox dan keyboard. |
| `/**` | `max_wz` | `1.0` | Batas yaw, rad/s. |
| `/**` | `linear_accel_limit`, `angular_accel_limit` | `1.0`, `1.0` | Batas percepatan m/s² dan rad/s². Pelepasan stick Xbox langsung mengikuti target nol. |
| `jaguar_unified_teleop` | `linear_speed_base`, `linear_speed_boost` | `0.7`, `1.0` | Batas gabungan kecepatan X/Y saat RT dilepas dan ditekan penuh, m/s. Tetap tunduk pada batas bersama. |
| `jaguar_cmd_vel_mux` | `xbox_timeout`, `keyboard_timeout` | `0.30`, `0.30` | Timeout heartbeat sumber command, detik. |
| `nxp_jaguar_controller` | `cmd_timeout` | `0.25` | Watchdog command kecepatan, detik. |
| `nxp_jaguar_controller` | `action_ema_alpha` | `0.0` | `<= 0`: bypass; jika aktif: `filtered = alpha * filtered_lama + (1-alpha) * action`. Untuk tuning gunakan `0 < alpha < 1`. |
| `nxp_jaguar_controller` | `walk_ramp_duration` | `0.8` | Durasi blend cosine dari pose berdiri ke target policy, detik; `<= 0` melewati ramp. |
| `nxp_jaguar_controller` | `rl_kp_roll`, `rl_kp_pitch`, `rl_kp_knee` | Masing-masing `45.0` | Gain Kp saat WALK, Nm/rad. |
| `nxp_jaguar_controller` | `rl_kd_roll`, `rl_kd_pitch`, `rl_kd_knee` | Masing-masing `4.5` | Gain Kd saat WALK, Nm/(rad/s). |
| `nxp_jaguar_controller` | `transition_kp_roll`, `transition_kp_pitch`, `transition_kp_knee` | Masing-masing `45.0` | Kp berdiri dan transisi biasa, Nm/rad. |
| `nxp_jaguar_controller` | `transition_kd_roll`, `transition_kd_pitch`, `transition_kd_knee` | Masing-masing `4.5` | Kd berdiri dan transisi biasa, Nm/(rad/s). |
| `nxp_jaguar_controller` | `imu_upside_down` | `true` | Transform IMU yang dipasang terbalik 180° pada sumbu X. |
| `nxp_jaguar_controller` | `use_imu_stabilization` | `true` | Auto-level selama STANDUP/STAND_HOLD, bukan WALK. |
| `nxp_jaguar_controller` | `imu_kp_pitch`, `imu_kd_pitch` | `0.55`, `0.04` | Gain koreksi pitch IMU. |
| `nxp_jaguar_controller` | `imu_kp_roll`, `imu_kd_roll` | `0.30`, `0.03` | Gain koreksi roll IMU. |
| `nxp_jaguar_controller` | `imu_hip_tolerance`, `imu_roll_tolerance` | `0.15`, `0.12` | Batas trim hip dan collar, rad. |
| `nxp_jaguar_controller` | `imu_tilt_deadband` | `0.02` | Deadband tilt IMU, rad. |
| `nxp_jaguar_controller` | `overtorque_threshold` | `14.0` | Ambang diagnostik feedback torque, tanpa park/E-stop, Nm; harus positif dan tidak melebihi ceiling RS00. Terpisah dari cap driver. |
| `nxp_jaguar_controller` | `safe_park_duration` | `2.0` | Durasi gerakan park, detik; positif. |
| `nxp_jaguar_controller` | `safe_park_kp`, `safe_park_kd` | `14.0`, `0.5` | Gain saat safe park. |
| `nxp_jaguar_controller` | `safe_park_double_press_window` | `1.0` | Jendela permintaan park berulang, detik. |
| `nxp_jaguar_controller` | `shutdown_settle_delay` | `0.5` | Waktu hold setelah safe park sebelum kembali pasif, detik. |
| `controller_manager` | `update_rate`, `thread_priority` | `200`, `80` | Frekuensi ros2_control read/update/write (Hz) dan prioritas thread manager. |
| `jaguar_impedance_controller` | `command_timeout` | `0.10` | Timeout command aktif dalam detik; monotonic clock, memicu E-stop jika stream hilang. |
| `jaguar_hardware` | `control_torque_limit_nm` | `14.0` | Cap torque software/firmware C++; valid `0 < nilai <= 14`. |
| Kedua node driver CAN | `default_coxa_kp`, `default_kp`, `default_knee_kp` | Masing-masing `45.0` | Gain awal collar, hip, knee. Command controller membawa gain tersendiri. |
| Kedua node driver CAN | `default_coxa_kd`, `default_kd`, `default_knee_kd` | Masing-masing `4.5` | Damping awal collar, hip, knee. |
| `jaguar_can_hardware` | `rate_hz` | `200` | Frekuensi driver Python ketika fallback dipilih. |
| `joy_node` | `dev`, `deadzone`, `autorepeat_rate` | `/dev/input/js0`, `0.10`, `50.0` | Nilai yang dikirim ke joy opsional, bukan pembaca Xbox langsung. Nama parameter perangkat bergantung implementasi joy yang terpasang. |

Pada branch ini launcher bersama memakai `controller_manager` dan plugin
`JaguarSystem`. Section `jaguar_hardware` diterjemahkan ke hardware parameters
pada URDF; optional `motor_params_file` dapat ditambahkan di section tersebut.
Section driver standalone lama tetap tersedia untuk tool terpisah.
`torque_safety_enabled` dihapus; overtorque selalu diagnostik, termasuk saat park.
Lihat [arsitektur dan matriks safety ros2_control](ros2_control.md).

Parameter lama `shutdown_duration` sudah dihapus karena tidak digunakan jalur
kontrol. Durasi park ditentukan oleh `safe_park_duration`.

## Pengaturan di kode, environment, atau file terpisah

| Pengaturan | Lokasi / default | Cara mengubah |
| --- | --- | --- |
| Pose berdiri | `scripts/parameters.py`: `STAND_ANGLE`; collar `0`, hip `-1.59`, knee depan `1.40`, belakang `1.38` rad | Edit pose dalam urutan ROS BL, BR, FL, FR; build dan restart. Bukan flag/YAML runtime. |
| Referensi nominal model 29996 | `DEFAULT_JOINT_POS` di controller, `DEFAULT_ANGLE` di parameters | Kontrak model nominal; model zero memakai array nol sebagai reference. |
| Action scale | `ACTION_SCALE=0.25` di controller | Kontrak training model, bukan parameter tuning launch. |
| Frekuensi controller / history | `CONTROL_DT=0.02` (50 Hz), history 5 × 45 | Kontrak runtime/model di kode; berbeda dari `rate_hz` driver. |
| Durasi stand/sit | `transition_duration=4.0` detik di controller | Saat ini konstanta kode. |
| Sensor timeout | `0.25` detik di controller | Saat ini konstanta kode; berbeda dari `cmd_timeout`. |
| Debounce overtorque controller | `torque_overload_cycles=5` | Saat ini konstanta kode; 100 ms pada 50 Hz. |
| Mapping CAN, arah motor, encoder offset, batas posisi | `scripts/parameters.py`, `scripts/joint_layout.py`, serta konfigurasi C++ | Kalibrasi/mapping di kode harus sinkron antara Python dan C++. |
| Thread inference CPU | Environment `JAGUAR_TORCH_THREADS`, default `1` | Set sebelum launch, misalnya `JAGUAR_TORCH_THREADS=2 ros2 launch ...`. |
| Kalibrasi gamepad | `$XDG_CONFIG_HOME/jaguar_control/gamepad_calibration.json`, fallback `~/.config/...`, kemudian default package | Gunakan tool kalibrasi Xbox. |
| Port IMU | Node `serial_imu_talker`: `port=/dev/ttyUSB0`, `frame_id=imu_link`, `topic_name=/Imu_data` | Parameter node tersedia, tetapi launcher bersama saat ini tidak meneruskan `config_file` ke IMU dan belum menyediakan flag untuk ketiganya. |

Snapshot `models/.../env.yaml` dan `agent.yaml` merekam konfigurasi training.
Mengeditnya tidak mengubah policy TorchScript yang sudah diekspor maupun
parameter runtime ROS.

## Contoh pemakaian

Konfigurasi runtime sendiri, dengan domain DDS yang eksplisit:

```bash
cp config/sim2real.yaml /tmp/jaguar-zero.yaml
# Edit nilai pada section node yang sesuai, misalnya walk_ramp_duration: 1.2.
ros2 launch jaguar_control sim2real_zero.launch.py \
  config_file:=/tmp/jaguar-zero.yaml domain_id:=1
```

Memilih export zero-reference lain:

```bash
ros2 launch jaguar_control sim2real_zero.launch.py \
  policy_path:=/path/absolut/custom_zero_policy.pt
```

Memeriksa argumen tanpa memulai hardware atau controller:

```bash
ros2 launch jaguar_control sim2real_zero.launch.py --show-args
```

Untuk menjalankan stack dengan feedback dari sumber lain, pilih
`with_hardware:=false`; matikan pula `with_imu`/`with_teleop` bila sumber tersebut
juga eksternal. Controller tetap memerlukan feedback yang valid sebelum WALK.


## Kontrak model 19999

Artifact dan manifest berada di `models/adaboot_l2c2_zero_reference_clearance5cm_19999_20261111/` dan
`models/deployment_adaboot_l2c2_zero_reference_clearance5cm_19999_20261111.json`. Input adalah history
`[batch,5,45]` time-major; estimator memakai packing per term, lalu mean actor
di-clip +/-10 di dalam TorchScript sebelum menjadi feedback aksi dan target
sendi. Referensi zero dan mode history dipertahankan setelah reset controller.
Model 5999 lama tetap tersedia, sementara model nominal tetap 29996.

Adaptasi ini berbasis main `4c57327` (migrasi ros2_control dan kalibrasi rear hip
terbaru). Batas torsi 14 Nm, mapping dan kalibrasi dipertahankan dari main.
Reward training memakai tinggi badan 25 cm / weight -10 dan clearance 5 cm
simetris dikalikan magnitudo kecepatan XY kaki; tidak ada input privileged saat
inferensi. Ini bukan jaminan bahwa lintasan kaki selalu tepat 5 cm.
