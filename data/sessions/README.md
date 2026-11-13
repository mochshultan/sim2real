# Recorded zero-reference sessions

These archives contain complete controller session logs copied from the robot's
local session directory. Original local logs remain in place. Each `.jsonl.gz`
is a lossless gzip archive; its matching `.summary.json` records model identity,
repository commit at archival, timestamps, record counts, and SHA-256 checksums for both the
archive and its uncompressed bytes.

## Pion 11500 — 2026-11-13 12:05:58 UTC (20:05:58 Taipei)

- Archive: [`sim2real_zero_20261113T120558.809300Z_86e4df08.jsonl.gz`](sim2real_zero_20261113T120558.809300Z_86e4df08.jsonl.gz)
- Summary: [`sim2real_zero_20261113T120558.809300Z_86e4df08.summary.json`](sim2real_zero_20261113T120558.809300Z_86e4df08.summary.json)
- Duration: 74.941 seconds; 3,748 samples; 3,073 policy evaluations in WALK.
- 3,670 samples include published commands for all 12 motors.
- Normal recording shutdown; zero dropped samples; all JSON lines parsed.
- Standing target: roll 0, hip -1.25, knee +1.25 rad for all legs.

The metadata line defines observation layout, scales and joint order. Samples
contain observation/state, full policy history during inference, policy actions,
clamped motor position/velocity commands and Kp/Kd, plus measured feedback.
A null motor output means no command was published during that sample. A null
policy action means inference did not run. Kp/Kd are gains, not measured torque.
See [the recording format](../../docs/zero_reference.md#log-otomatis-per-sesi).

Read without extracting:

```bash
gzip -cd data/sessions/sim2real_zero_20261113T120558.809300Z_86e4df08.jsonl.gz | less
```

Record counts describe captured controller data; they do not establish tracking
quality or a hardware safety assessment.
