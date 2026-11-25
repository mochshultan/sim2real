# Pion DreamWaQ VHIP mean-context checkpoint9999

Source: `2026-11-24_23-31-42_ppo-vhip-contextmean-fresh10000-seed42-8192env-gpu0-20261124-175956`.

- `model_9999.pt`: full checkpoint after10000 fresh PPO updates; no L2C2.
- `env.yaml`, `agent.yaml`: exact saved training configuration.
- `policy.pt`: raw training export with policy45/history225 inputs.
- `policy_pion_qr1_dreamwaq_vhip_mean_native_9999_20261125.pt`: controller export, input `[batch,5,45]`, output `[batch,12]`, clip±10.
- Paired `deployment_*.json`: hashes, ABI, action contract and validation results.

Actor context is the CENet posterior mean during both training and export.
Native Pion action order: FL/FR/BL/BR collar, then hip, then knee. Runtime uses
current measured joint position +0.25*action at50Hz, Kp45/Kd4.5. This named run
uses the original Pion training plant, not the newly measured20261124 URDF.
The model is a temporary deployment candidate; offline correspondence passed,
without a hardware motion test. Prior model bundles are retained.

The default `ros2 launch jaguar_control sim2real_64d.launch.py` includes
`sim2real_curpos.launch.py`, which now selects this native export after rebuild.
Do not feed the raw two-input policy to the native controller.
