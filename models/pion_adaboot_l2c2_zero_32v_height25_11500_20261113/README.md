# Pion AdaBoot-L2C2 11500, zero-reference deployment

The zero-reference launcher selects the versioned TorchScript policy for this
checkpoint. The ordinary nominal launcher retains model 39996. The runtime is
based on main `9488fb1`; its calibration, standing pose, hardware mappings,
gains and 14 Nm cap are preserved.

## Contracts

- Source: PC2 `pion_qr1_adaboot_l2c2_zero_32v_height25/2026-11-13_13-35-29_from5300-h25-w10-std1-to11500-20261113-1152env/model_11500.pt`.
- Input: time-major `[batch,5,45]`, oldest first; output `[batch,12]`.
- Zero joint reference, scale 0.25, repeat-history cold start.
- Velocity estimator and normalized actor only; no privileged critic or simulator velocity input.
- Estimated linear 32 V RS00 actuator profile in training; height target 25 cm,
  weight -10, directional clearance target 5 cm, standing commands 15%.
- Deterministic mean actions clipped to +/-10 before controller target/action feedback.

Training order is FL/FR/BL/BR within collar, hip, then knee groups. The existing
controller order is FR/FL/BR/BL within those groups. `PionJointOrderPolicy`
reorders all position, velocity and last-action blocks of every frame before
estimator packing, then reorders actions back. Body IMU and command fields
retain their indices. Only permutation is applied; encoder signs, joint scales,
nominal reference and mechanical calibration are not changed by the adapter.

The adjacent JSON manifest records hashes, both joint orders, complete index
permutations, and CPU test cases. `env.yaml` and `agent.yaml` are the exact
training snapshots. The source checkpoint and prior Jaguar exports remain
available under versioned filenames.

## Validation

Export output matched the native Pion training actor after independent
anatomical joint mapping exactly on nine cases: batch 1/16/64, each with
zero/random/stress histories. Maximum absolute difference was 0.0. A regression
test exercises all joint-history blocks and actions using anatomical-name
lookups independent of the adapter's index buffers.

All 154 repository Python tests passed in the uv-managed offline environment.
All four native CTests passed. The combined five-entry CTest suite also passed
using the same isolated Torch/python-can/pytest dependencies exposed through
PYTHONPATH; system packages were not changed. The default system Python on
this development PC lacks those optional runtime dependencies.

The ROS2 package built successfully. The installed zero launch arguments
resolve the current policy, `policy_reference=zero`, and
`history_initialization=repeat`. Source and installed policy hashes match.
The nominal `models/policy.pt` hash remains
`bc37ca24b38f2d659cfcccb9d1101219cdc5dc576003ebcd2dd5a4049d48ddcb`.

No CAN or physical robot motion was started during validation. CPU policy
parity and build/launch checks do not establish hardware locomotion results.

## Re-export

From an environment with PyTorch:

```bash
python scripts/export_adaboot_policy.py \
  --checkpoint models/model_pion_adaboot_l2c2_zero_32v_height25_11500_20261113.pt \
  --tag pion_adaboot_l2c2_zero_32v_height25_11500_20261113 \
  --clip-actions 10 --training-joint-order pion
```

Regenerate and validate the manifest if export bytes change. Do not use
`--set-default` for this zero policy; the ordinary launcher selects the nominal
model through `models/policy.pt`.
