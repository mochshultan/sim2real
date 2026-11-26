# Pion measured-URDF MoE-Loco L2C2 checkpoint4999

Source run: `2026-11-25_17-44-28_moeloco-measured-vhip-oracleprior-ramp01-contextmean-l2c2-oracle2k-adapt3k-fresh-seed42-8192env-gpu0-20261125-174426`.

Full checkpoint after2000 oracle +3000 adaptation updates, L2C2, oracle-centred
KL and AdaBoot max increase0.01. The deployed model uses CENet posterior mean;
no contact oracle, decoder, privileged critic or recurrent state is required.
Four actor experts512/256/128 are mixed by a64/128/64/4 softmax router.

`policy.pt` is the reconstructed training-class export (policy45/history225).
`policy_pion_qr1_moeloco_measured_native_4999_20261126.pt` is the ROS controller export, input `[batch,5,45]`,
output12 actions clipped±10. Paired deployment manifest records hashes, input
contract and parity checks. `model_4999.pt`, `env.yaml`, `agent.yaml` and asset
manifests retain training provenance. Asset manifests describe conversion;
they do not establish physical hardware calibration.

Joint order is FL/FR/BL/BR collar, then hip, then knee. Runtime uses measured
position +0.25*action at50Hz, Kp45/Kd4.5. Use `sim2real_moe.launch.py` after
building the ROS package. The existing PPO launcher stays unchanged. Export
correspondence is checked offline; no hardware motion test was performed.
