# 🤖 Robot Models & Trained Neural Network Weights (`models/`)

This directory contains robot description models and trained Reinforcement Learning (RL) policy weights for the **NXP Jaguar Quadruped**.

---

## 📁 Contents

### 🧠 Trained Policy Checkpoints
- **`policy.pt`**: Production TorchScript JIT actor policy model (DreamWaQ architecture, 48-D observation $\\to$ 12-D actions).
- **`policy.onnx`**: Exported ONNX model format for TensorRT or OpenVINO inference.
- **`policy_2100.pt` / `model_2100.pt`**: Intermediate training checkpoint weights (Iteration 2100).

### 🤖 Robot Kinematic Models
- **`nxp_jaguar.urdf`**: URDF model specifying link dimensions, joint axes, inertial properties, and visual/collision mesh references.
- **`meshes/`**: 3D CAD STL mesh files for base body and leg linkages.
