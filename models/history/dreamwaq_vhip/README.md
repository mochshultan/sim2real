# Pion DreamWaQ VHIP checkpoint 5000

Source run: `2026-11-23_16-41-21_dreamwaq-vhip-entropy002-clearance3cm-no_l2c2-fresh10k-seed42-8192env-gpu0-20261123-164119`.

- `model_5000.pt`: full resumable training checkpoint.
- `policy.pt`: raw training TorchScript export, inputs `[batch,45]` and `[batch,225]`.
- `policy.onnx` and `policy.onnx.data`: the same raw input contract in ONNX.
- `policy_pion_qr1_dreamwaq_vhip_native_5000_20261123.pt`: native ROS controller export, input `[batch,5,45]`, output `[batch,12]`, action clipping ±10.
- `deployment_pion_qr1_dreamwaq_vhip_native_5000_20261123.json`: paired runtime manifest, hashes, source and correspondence records.
- `env.yaml` and `agent.yaml`: exact saved training configuration.

Use `ros2 launch jaguar_control sim2real_64d.launch.py` after rebuilding and
sourcing the robot workspace. The 64D launcher selects this native export.
The raw two-input export is retained for training-side comparisons and is not
the ROS controller ABI. Policy action order is FL/FR/BL/BR collar, hip, knee;
targets use measured position plus 0.25 times action at 50 Hz, Kp45/Kd4.5.

This checkpoint uses DreamWaQ without L2C2, adaptive entropy initially 0.02,
VHIP angle/angular-acceleration and actual-speed-gated stand-still pose rewards,
19 cm base-height target, and 3 cm DreamWaQ clearance. Training continues in
its original experiment; this is an intermediate checkpoint, not a completed
10k run. Export correspondence was checked offline; hardware motion was not tested.
