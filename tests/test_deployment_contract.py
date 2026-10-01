"""Regression tests for the pinned sim2real model and actuator gains."""

import hashlib
from pathlib import Path
import unittest

import yaml


ROOT = Path(__file__).parents[1]
POLICY_SHA256 = "6dc1e6e41ae576cc4ffacb8917268fb4f5c5f43aa169fbcd6db350be48e920fe"
CHECKPOINT_SHA256 = "a4a555fc55e500938369a0a0d180847d4364f2c08f2338cd1a11e710324f7a6d"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


class DeploymentContractTests(unittest.TestCase):
    def test_default_and_archived_policy_are_model_9999_export(self):
        default_policy = ROOT / "models/policy.pt"
        archived_policy = ROOT / "models/policy_adaboot_ideal_l2c2_9999_20261001.pt"
        checkpoint = ROOT / "models/model_adaboot_ideal_l2c2_9999_20261001.pt"

        self.assertEqual(_sha256(default_policy), POLICY_SHA256)
        self.assertEqual(_sha256(archived_policy), POLICY_SHA256)
        self.assertEqual(_sha256(checkpoint), CHECKPOINT_SHA256)

    def test_launch_and_controller_use_deterministic_default_policy(self):
        launch_source = (ROOT / "launch/sim2real.launch.py").read_text()
        controller_source = (ROOT / "scripts/nxp_jaguar_controller.py").read_text()

        self.assertIn('os.path.join(pkg_dir, "models", "policy.pt")', launch_source)
        self.assertNotIn("policy_*.pt", launch_source)
        self.assertNotIn("policy_*.pt", controller_source)

    def test_sim2real_gains_match_training(self):
        config = yaml.safe_load((ROOT / "config/sim2real.yaml").read_text())
        controller = config["nxp_jaguar_controller"]["ros__parameters"]
        hardware_nodes = ("robstride_can_hardware", "jaguar_can_hardware")

        for mode in ("rl", "transition"):
            for joint in ("roll", "pitch", "knee"):
                self.assertEqual(controller[f"{mode}_kp_{joint}"], 40.0)
                self.assertEqual(controller[f"{mode}_kd_{joint}"], 1.0)

        for node in hardware_nodes:
            hardware = config[node]["ros__parameters"]
            for prefix in ("default_coxa", "default", "default_knee"):
                self.assertEqual(hardware[f"{prefix}_kp"], 40.0)
                self.assertEqual(hardware[f"{prefix}_kd"], 1.0)

        self.assertEqual(controller["safe_park_kp"], 14.0)
        self.assertEqual(controller["safe_park_kd"], 0.5)

        for config_name in ("nxp_jaguar_controller.yaml", "robstride_can.yaml"):
            config = yaml.safe_load((ROOT / "config" / config_name).read_text())
            for node in config.values():
                parameters = node.get("ros__parameters", {})
                for name, value in parameters.items():
                    if name.endswith("_kd") and name.startswith(("rl_", "transition_", "default")):
                        self.assertEqual(value, 1.0, f"{config_name}: {name}")

        sim2sim = yaml.safe_load((ROOT / "config/sim2sim_mujoco.yaml").read_text())["sim2sim_mujoco"]
        self.assertEqual(sim2sim["walk_gains"]["coxa_kd"], 1.0)
        self.assertEqual(sim2sim["walk_gains"]["pitch_kd"], 1.0)
        self.assertEqual(sim2sim["standup_gains"]["kd"], 1.0)


if __name__ == "__main__":
    unittest.main()
