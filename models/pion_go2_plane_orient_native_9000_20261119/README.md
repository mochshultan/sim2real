# Pion Go2 plane_orient checkpoint 9000

Copied from PC2 while its GPU1 training continued. The exact checkpoint,
export hashes, source run and CPU parity results are recorded in
[`../deployment_pion_go2_plane_orient_native_9000_20261119.json`](../deployment_pion_go2_plane_orient_native_9000_20261119.json).

- Checkpoint iteration 9000; seed 42; 4096 environments.
- Newton MJWarp: physics dt 0.0025 s, decimation 8, policy period 0.02 s.
- Native FL/FR/BL/BR order in each collar/hip/knee joint group.
- MLP actor 48 -> 512 -> 256 -> 128 -> 12; estimator 225 -> 128 -> 64 -> 3.
- Input `[B,5,45]`, term-major estimator packing; estimator-only deployment.
- Current-position targets: q_measured + 0.25 * clip(action, -10, 10).
- Training orientation reward -5 relative to the last valid support plane.
  At least three terrain contacts refresh the plane; at most two retain it.
  Episode reset clears the reference. The archived metric name is
  `support_plane_orientation_l2`; subsequent tasks call it `plane_orient`.
- Base-height target 0.20 m, weight -25; standing airborne weight -1.
- Export matched the reconstructed native checkpoint actor exactly on CPU for
  batches 1/16/64 and zero/random/stress histories. Controller telemetry inference
  matched as well. No physical robot motion or simulation rollout was tested.

The plane is reward-only information, not an actor input. The runtime controller
needs no additional plane sensor. Checkpoint 5000 remains available separately.
`env.yaml` and `agent.yaml` are the original saved training configurations.
