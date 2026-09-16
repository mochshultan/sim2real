#!/usr/bin/env python3
"""Export adaptive TorchScript policy for sim2real deployment."""

import os
import torch
import torch.nn as nn

class AdaptiveBaselinePolicy(nn.Module):
    def __init__(self, base_policy: nn.Module):
        super().__init__()
        self.base_policy = base_policy

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        # If 3D temporal history buffer (e.g. [1, 5, 45]), slice latest observation
        if x.dim() == 3:
            obs = x[:, -1, :]
        elif x.dim() == 1:
            obs = x.unsqueeze(0)
        else:
            obs = x
        return self.base_policy(obs)

class PyTorchBaselinePolicyWrapper(nn.Module):
    def __init__(self, actor_state_dict: dict[str, torch.Tensor], eps: float = 0.01):
        super().__init__()
        self.register_buffer("mean", actor_state_dict["obs_normalizer._mean"].clone().cpu())
        self.register_buffer("std", actor_state_dict["obs_normalizer._std"].clone().cpu())
        self.eps = eps

        self.mlp = nn.Sequential(
            nn.Linear(45, 512),
            nn.ELU(),
            nn.Linear(512, 256),
            nn.ELU(),
            nn.Linear(256, 128),
            nn.ELU(),
            nn.Linear(128, 12),
        )
        mlp_sd = {k.replace("mlp.", ""): v.cpu() for k, v in actor_state_dict.items() if k.startswith("mlp.")}
        self.mlp.load_state_dict(mlp_sd)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x_norm = (x - self.mean) / (self.std + self.eps)
        return self.mlp(x_norm)


def main():
    import argparse
    import shutil

    parser = argparse.ArgumentParser(description="Export adaptive TorchScript and ONNX policy for sim2real")
    parser.add_argument(
        "--checkpoint",
        type=str,
        default="/home/shultan/IsaacLab/logs/rsl_rl/nxp_jaguar_baseline_tibia/2026-09-15_14-38-50/model_2999.pt",
        help="Path to training checkpoint model_*.pt",
    )
    parser.add_argument(
        "--raw-policy",
        type=str,
        default="/home/shultan/IsaacLab/logs/rsl_rl/nxp_jaguar_baseline_tibia/2026-09-15_14-38-50/exported/policy.pt",
        help="Path to raw TorchScript policy exported from Isaac Lab / RSL-RL",
    )
    parser.add_argument(
        "--tag",
        type=str,
        default="baseline_tibia_v2_2999",
        help="Model tag for versioned output filename",
    )
    parser.add_argument(
        "--models-dir",
        type=str,
        default="/home/shultan/jaguar_sim2real/models",
        help="Target models directory",
    )
    parser.add_argument(
        "--set-default",
        action="store_true",
        default=True,
        help="Also overwrite policy.pt and policy.onnx as default",
    )
    args = parser.parse_args()

    os.makedirs(args.models_dir, exist_ok=True)

    # 1. Load checkpoint and update raw policy weights if checkpoint exists
    raw_policy = None
    if os.path.isfile(args.checkpoint):
        print(f"[INFO] Loading checkpoint from: {args.checkpoint}")
        ckpt = torch.load(args.checkpoint, map_location="cpu")
        actor_sd = ckpt["actor_state_dict"]
        iter_num = ckpt.get("iter", "unknown")
        print(f"       -> Checkpoint iteration: {iter_num}")

        # Copy checkpoint to models directory
        shutil.copy2(args.checkpoint, os.path.join(args.models_dir, f"model_{args.tag}.pt"))

        # If raw_policy exists, update its state dict with checkpoint weights
        if args.raw_policy and os.path.isfile(args.raw_policy):
            raw_policy = torch.jit.load(args.raw_policy, map_location="cpu")
            clean_sd = {k: v for k, v in actor_sd.items() if k in raw_policy.state_dict()}
            raw_policy.load_state_dict(clean_sd)
            raw_policy.eval()

        # Build PyTorch model for ONNX and fallback TorchScript export
        py_model = PyTorchBaselinePolicyWrapper(actor_sd)
        py_model.eval()

        # Export ONNX (self-contained legacy exporter)
        onnx_tag_path = os.path.join(args.models_dir, f"policy_{args.tag}.onnx")
        torch.onnx.export(
            py_model,
            torch.zeros(1, 45),
            onnx_tag_path,
            input_names=["obs"],
            output_names=["actions"],
            opset_version=18,
            dynamo=False,
        )
        print(f"[SUCCESS] Exported ONNX model to: {onnx_tag_path}")
        if args.set_default:
            onnx_def_path = os.path.join(args.models_dir, "policy.onnx")
            shutil.copy2(onnx_tag_path, onnx_def_path)
            # Remove stale .data if standalone ONNX was generated
            stale_data = os.path.join(args.models_dir, "policy.onnx.data")
            if os.path.exists(stale_data):
                try:
                    os.remove(stale_data)
                except OSError:
                    pass
            print(f"[SUCCESS] Updated default ONNX model: {onnx_def_path}")

    if raw_policy is None:
        if args.raw_policy and os.path.isfile(args.raw_policy):
            raw_policy = torch.jit.load(args.raw_policy, map_location="cpu")
            raw_policy.eval()
        elif "py_model" in locals() and py_model is not None:
            raw_policy = torch.jit.trace(py_model, torch.zeros(1, 45, dtype=torch.float32))
            raw_policy.eval()
        else:
            raise FileNotFoundError(f"Policy file not found: {args.raw_policy}")

    # Save raw policy
    raw_tag_path = os.path.join(args.models_dir, f"policy_{args.tag}_raw.pt")
    torch.jit.save(raw_policy, raw_tag_path)

    # 2. Build adaptive wrapper
    wrapper = AdaptiveBaselinePolicy(raw_policy)
    scripted_policy = torch.jit.script(wrapper)
    scripted_policy.eval()

    # Verification tests
    dummy_3d = torch.randn(1, 5, 45)
    dummy_2d = dummy_3d[:, -1, :].clone()
    dummy_1d = dummy_2d.squeeze(0).clone()

    out_raw = raw_policy(dummy_2d)
    out_3d = scripted_policy(dummy_3d)
    out_2d = scripted_policy(dummy_2d)
    out_1d = scripted_policy(dummy_1d)

    print("Verification checks:")
    print("  out_raw shape:", tuple(out_raw.shape))
    print("  out_3d shape :", tuple(out_3d.shape))
    print("  out_2d shape :", tuple(out_2d.shape))
    print("  out_1d shape :", tuple(out_1d.shape))

    assert (out_raw - out_3d).abs().max().item() < 1e-6, "3D output mismatch!"
    assert (out_raw - out_2d).abs().max().item() < 1e-6, "2D output mismatch!"
    assert (out_raw - out_1d).abs().max().item() < 1e-6, "1D output mismatch!"

    versioned_path = os.path.join(args.models_dir, f"policy_{args.tag}.pt")
    torch.jit.save(scripted_policy, versioned_path)
    print(f"[SUCCESS] Exported adaptive policy to: {versioned_path}")

    if args.set_default:
        default_path = os.path.join(args.models_dir, "policy.pt")
        torch.jit.save(scripted_policy, default_path)
        print(f"[SUCCESS] Updated active default: {default_path}")

    print("\n[ALL DONE] Model deployment to sim2real completed successfully.")

if __name__ == "__main__":
    main()
