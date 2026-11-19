# Pion Go2 tracking checkpoint 5000

This snapshot was copied from PC2 while training continued. The exact checkpoint
and export hashes, source run, contracts and validation are recorded in
[`../deployment_pion_go2_tracking_native_5000_20261119.json`](../deployment_pion_go2_tracking_native_5000_20261119.json).

- Checkpoint iteration: 5000; seed: 42; environments: 4096.
- Training: Newton MJWarp, physics dt 0.0025 s, decimation 8, policy dt 0.02 s.
- Actor: 48 -> 512 -> 256 -> 128 -> 12, ELU, embedded normalization.
- Estimator: 225 -> 128 -> 64 -> 3; estimator-only deployment.
- Export: native Pion order, input `[B,5,45]`, action clipping at +/-10.
- Targets: measured joint position + 0.25 * clipped action, once per policy step;
  controller/driver position guards still apply.
- Training target bounds: collar [-0.7,0.7], hip [-2.0,1.0], knee [-0.05,1.6] rad.
- Go2 rough rewards: base height 0.20 m (weight -30), standing airborne weight -1.
- Exact CPU export parity against the reconstructed training actor: batches
  1/16/64 with zero/random/stress histories, maximum absolute difference 0.0.
- Controller telemetry inference also matched exactly. No physical robot motion
  or simulation rollout with this export was tested.

`env.yaml` and `agent.yaml` are the original training snapshots. The runtime
checkpoint is not required by the controller; it is retained for reproducibility.
Newton, MuJoCo and hardware physics are not numerically equivalent.
