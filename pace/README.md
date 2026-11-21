# PACE Pion current — data actuator dari robot asli

Folder ini menyiapkan rekaman untuk task PC1 `Isaac-Pace-Pion-Current-v0`.
Robot menjalankan ROS 2 Jazzy dan driver CyberGear/RS00 repo ini. Tidak ada
policy RL atau IsaacLab yang dijalankan di robot selama chirp.

| File | Fungsi |
|---|---|
| `run_pion_chirp.py` | Preview, lalu chirp fisik FR hanya bila memakai `--arm` |
| `record.sh` | Rekam feedback dan command ke rosbag SQLite; tidak menggerakkan motor |
| `driver.yaml` | Driver standalone 200 Hz, gain 45/4.5, batas torque 3 Nm |
| `convert_to_pace.py` | Konversi bag menjadi `.pt` untuk fitting PACE di PC1 |

## Profil pengukuran

Robot **digantung dengan semua kaki bebas** selama transisi dan chirp. Script
menahan sembilan joint lain pada pose nominal collar/hip/knee `0/-1.1/+1.1 rad`,
lalu memberi chirp pada FR collar, hip, dan knee: amplitudo maksimal 0.03 rad,
0.1–0.8 Hz selama 20 s pada command 50 Hz. Transisi ke pose nominal memerlukan
18 s. Profil `validation` membalik sweep menjadi 0.8–0.1 Hz; rekam sebagai run
terpisah dari `fit`.

Gain seluruh joint saat chirp **Kp 45, Kd 4.5**. Desired velocity dan feed-forward
torque adalah nol, sesuai replay PD PACE. Kecepatan perubahan target hanya
digunakan untuk memeriksa batas gerak. Target adalah **posisi absolut**, bukan
raw action policy. Pada policy current, target asalnya `q_measured + 0.25 * action`;
jangan tambahkan posisi terukur lagi saat replay.

Tidak memakai IMU leveling atau pose Jaguar lama. Joint limits mengikuti Pion
IsaacLab 3: collar `[-0.7,0.7]`, hip `[-2,1]`, knee `[-0.05,1.6]` rad.

## Menjalankan di robot

Pull repo, build jika driver belum tersedia, lalu source ROS dan workspace
pada **setiap terminal**. Gunakan path workspace robot milikmu:

```bash
git pull --ff-only origin main
source /opt/ros/jazzy/setup.bash
source /path/ke/workspace/install/setup.bash
cd /path/ke/sim2real
```

Hentikan launch sim2real/policy, simulator, dan publisher command lain sebelum
memulai. Jangan menjalankan driver standalone bersamaan dengan ros2_control;
hanya satu proses yang boleh memiliki bus CAN. Script menolak publisher command
lain, feedback dari beberapa driver, data stale, dan torque limit selain 3 Nm.
Pastikan robot digantung dan emergency stop tersedia. Pengaturan 3 Nm perlu
terkonfirmasi pada driver/firmware; laporan ROS saja bukan bukti readback firmware.

Preview tanpa ROS dan tanpa motor:

```bash
python3 pace/run_pion_chirp.py --profile fit
```

**Terminal A — mulai recording dahulu:**

```bash
bash pace/record.sh pion_fit_01
```

**Terminal B — mulai script chirp:**

```bash
python3 pace/run_pion_chirp.py --profile fit --label fit_01 --arm
```

Script menunggu driver maksimal 60 s sambil mengirim heartbeat gain nol.

**Terminal C — jalankan hanya driver standalone:**

```bash
ros2 launch jaguar_control robstride_standalone.launch.py \
  config_file:="$(pwd)/pace/driver.yaml"
```

Sesudah preflight lolos ada countdown 5 s, transisi 18 s, settling 5 s,
lalu chirp 20 s. Script mengembalikan gain nol saat selesai. Ctrl+C pada script
mengirim gain nol; pelanggaran guard selama gerak mengirim emergency stop.
Guard feedback mencakup timeout 0.1 s, torque 2.5 Nm, velocity 0.30 rad/s,
tracking error 0.05 rad, dan temperature 65 C. Ini batas profil pengukuran,
bukan kalibrasi motor.

Setelah script selesai, hentikan recording dengan Ctrl+C, lalu hentikan driver.
Ulangi A/B/C dengan label berbeda dan `--profile validation --label validation_01`.
Jangan reset encoder-zero atau mengubah payload/gain di antara dua rekaman.

## File hasil dan konversi

Bag, command CSV, dan metadata JSON ditulis ke `data/pace/` (diabaikan Git).
Simpan tiga artefak ini bersama. Command CSV mencakup transisi; converter hanya
mengambil interval `fr_chirp` dari run yang selesai. Metadata memuat gain,
profil, pose, joint order, torque limit dan waktu chirp.

Konversi di PC yang memiliki ROS 2 Python, NumPy, dan PyTorch. Di PC1 source
ROS Jazzy lalu aktifkan environment IsaacLab yang memiliki PyTorch; pastikan
`python -c 'import rosbag2_py, rclpy, torch'` berhasil. Copy bag/CSV/JSON dari
robot tanpa mengubah timestamp.

```bash
python pace/convert_to_pace.py data/pace/pion_fit_01_TIMESTAMP \
  --command-csv data/pace/pion_chirp_fit_01_TIMESTAMP_commands.csv \
  --output data/pace/pion_fit_01.pt
```

Output: `time [T]`, `dof_pos [T,12]`, `des_dof_pos [T,12]`, `joint_order`, metadata.
Target di-zero-order-hold; posisi diinterpolasi ke 400 Hz untuk physics PACE
0.0025 s. Feedback driver tetap 200 Hz: interpolasi tidak menambah bandwidth
sensor. Gap/stale data di dalam window ditolak, bukan dihapus diam-diam.
Timestamp yang dipakai adalah waktu penerimaan bag ROS, bukan timestamp CAN;
hasil delay fitting dapat mencakup transport/scheduling ROS.

Urutan kolom PACE identik dengan action Pion IsaacLab 3:

```text
FL_collar_joint FR_collar_joint BL_collar_joint BR_collar_joint
FL_hip_joint    FR_hip_joint    BL_hip_joint    BR_hip_joint
FL_knee_joint   FR_knee_joint   BL_knee_joint   BR_knee_joint
```

Hardware memakai urutan BL/BR/FL/FR per kaki; converter memetakan berdasarkan
nama, termasuk alias lama `Fr_roll_joint` dan `Fr_hip_pitch_joint`.

## Fitting di PC1

```bash
conda activate isaaclab
cd ~/pace-sim2real
python scripts/pace/fit.py --viz none \
  --task Isaac-Pace-Pion-Current-v0 --num_envs 32 \
  --data /path/ke/pion_fit_01.pt --max_iterations 200
```

Bandingkan kandidat nominal dan fitted pada rekaman `validation`, menggunakan
`fit.py --evaluate_candidate ... --data /path/ke/pion_validation_01.pt`.
PACE saat ini fit tiga joint FR saja. Hasil bukan identifikasi joint lain.
Tidak ada script di folder ini yang memasang parameter fitted ke controller.
Batas pengukuran driver 3 Nm berbeda dari model training 14 Nm; data mendekati
saturasi tidak boleh dianggap cocok dengan model nominal tanpa evaluasi.
