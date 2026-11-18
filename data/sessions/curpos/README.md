# Log otomatis sim2real curpos

Setiap startup controller dari `scripts/sim2real_curpos` atau
`ros2 launch jaguar_control sim2real_curpos.launch.py` membuat folder unik:

```text
data/sessions/curpos/sim2real_curpos_<UTC>_<id>/telemetry.jsonl
```

Sim2sim memakai prefix `sim2sim_curpos`. Folder default ditentukan oleh launch
di repo, tidak bergantung pada working directory terminal. Lokasi dapat diubah
dengan `session_log_dir:=/path/tujuan`.

JSONL berisi metadata, sampel setiap siklus kontrol (50 Hz, termasuk saat
menunggu sensor/standby), event emergency stop, dan summary saat shutdown normal.
Metadata menjelaskan urutan joint, layout, skala observasi, model dan run ID.
Setiap sampel memiliki `wall_time_ns` (Unix UTC), `monotonic_time_ns`, serta
`ros_time_ns`; inferensi memiliki timestamp mulai/selesai UTC dalam nanosekon.

| Field | Isi |
| --- | --- |
| `observation` | Semua 45 nilai observasi saat ini |
| `policy_input` / `policy_history` | History 5×45, frame tertua ke terbaru |
| `estimator_input` | 225 nilai history yang dipacking per term observasi |
| `estimator_output` | 3 nilai estimasi berskala training; pada Pion, bagi `estimator_target_scale=2.0` untuk m/s |
| `actor_observation` | Observasi saat ini + estimasi kecepatan, 48 nilai |
| `actor_input` | Input policy actor setelah normalisasi, 48 nilai |
| `actor_output` | Output actor sebelum clipping, 12 nilai |
| `policy_output` / `policy_action` | Aksi akhir setelah clipping, 12 nilai |
| `motor_output` | Perintah posisi, kecepatan, Kp/Kd setelah limit kontrol |
| `feedback_*`, `body_*`, `command_velocity` | Feedback joint, IMU dan command controller |

Tensor internal dicatat dari satu evaluasi estimator/actor yang menghasilkan aksi
kontrol. Field inferensi bernilai `null` saat policy tidak dievaluasi; output motor
`null` saat tidak ada perintah yang dipublish. Validitas dan umur sensor dicatat
di setiap sampel. Ini log pada frekuensi kontrol, bukan rekaman semua pesan DDS.

Writer berjalan di thread terpisah dengan antrean terbatas agar disk tidak
memblokir kontrol. Summary memuat `written_records` dan `dropped_samples`;
controller memberi peringatan jika sampel terbuang atau writer gagal. Shutdown
paksa dapat meninggalkan log tanpa summary.

File hasil di folder ini tidak diignore Git. Tambahkan hasil yang ingin dikirim
dengan `git add data/sessions/curpos/`, lalu commit dan push seperti biasa.

## Per-motor CAN feedback age

New recordings include `can_feedback_age_s`, a 12-value list in the metadata's
`joint_names` order. Its origin is the host-monotonic reception time of the last
CAN feedback frame decoded for each motor, rather than the time a new DDS packet
reached the policy callback. `null` means reception time is unknown, including
mock hardware and the MuJoCo bridge. A stale CAN timestamp keeps ageing even if
the broadcaster continues to publish fresh DDS packets.

Both `ros2_control` (through the joint-state broadcaster) and the standalone CAN
node publish `can_rx_time_s` on `/dynamic_joint_states`. Hardware and policy must
run on the same Linux host because this timestamp uses its monotonic clock.
This does not measure firmware encoder sampling time or hardware bus arrival
before the driver reads/decodes a frame.

`can_rx_time_monotonic_s` retains raw timestamps. `can_timing_message_age_s`
measures time since that timing-message callback. Compare
`can_timing_source_ros_time_ns` and `joints_source_ros_time_ns` to see whether the
latest timing and position snapshots came from different publication cycles.
The age is evaluated before inference; inference and downstream command latency
are additional. The current JSONL recorder remains asynchronous and does not
write to disk in the CAN control loop.
