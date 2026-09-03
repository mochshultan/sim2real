# Robot Models and Policy Weights (`models/`)

Robot descriptions and trained reinforcement learning policy weights for the NXP Jaguar quadruped.

## Policy Checkpoints

- `policy.pt`: TorchScript JIT actor policy model (DreamWaQ, 48-D observation to 12-D actions).
- `policy.onnx`: Exported ONNX model for TensorRT or OpenVINO inference.
- `policy_2100.pt`, `model_2100.pt`: Training checkpoint weights at iteration 2100.

## Kinematic Models

- `nxp_jaguar.urdf`: URDF model with link dimensions, joint axes, inertial properties, and mesh references.
- `meshes/`: STL mesh files for chassis and leg linkages.
