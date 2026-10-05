"""Regression tests for the pinned sim2real model and actuator gains."""

import hashlib
from pathlib import Path
import unittest

import yaml


ROOT = Path(__file__).parents[1]
POLICY_SHA256 = "0bccb398f5e8f7bb61e5d6fe57ade6bc9877f0a3bbbdef3dd2915181983e496d"
CHECKPOINT_SHA256 = "d086be1e849f65dde83f5bfe8a5a9d4ac2b538e1821e875a046a188bada799e1"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


class DeploymentContractTests(unittest.TestCase):
    def test_default_and_archived_policy_are_model_9999_export(self):
        default_policy = ROOT / "models/policy.pt"
        archived_policy = ROOT / "models/policy_adaboot_ideal_l2c2_9999_20261004.pt"
        checkpoint = ROOT / "models/model_adaboot_ideal_l2c2_9999_20261004.pt"

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
                self.assertEqual(controller[f"{mode}_kd_{joint}"], 2.5)

        for node in hardware_nodes:
            hardware = config[node]["ros__parameters"]
            for prefix in ("default_coxa", "default", "default_knee"):
                self.assertEqual(hardware[f"{prefix}_kp"], 40.0)
                self.assertEqual(hardware[f"{prefix}_kd"], 2.5)

        self.assertEqual(controller["safe_park_kp"], 14.0)
        self.assertEqual(controller["safe_park_kd"], 0.5)

        for config_name in ("nxp_jaguar_controller.yaml", "robstride_can.yaml"):
            config = yaml.safe_load((ROOT / "config" / config_name).read_text())
            for node in config.values():
                parameters = node.get("ros__parameters", {})
                for name, value in parameters.items():
                    if name.endswith("_kd") and name.startswith(("rl_", "transition_", "default")):
                        self.assertEqual(value, 2.5, f"{config_name}: {name}")

        sim2sim = yaml.safe_load((ROOT / "config/sim2sim_mujoco.yaml").read_text())["sim2sim_mujoco"]
        self.assertEqual(sim2sim["walk_gains"]["coxa_kd"], 2.5)
        self.assertEqual(sim2sim["walk_gains"]["pitch_kd"], 2.5)
        self.assertEqual(sim2sim["standup_gains"]["kd"], 2.5)

    def test_controller_nominal_pose_matches_training(self):
        source = (ROOT / "scripts/nxp_jaguar_controller.py").read_text()
        self.assertIn("-1.51, -1.51, -1.49, -1.49", source)
        self.assertIn("1.22,  1.22,  1.20,  1.20", source)


if __name__ == "__main__":
    unittest.main()
