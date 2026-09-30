"""Regression tests for the pinned sim2real model and actuator gains."""

import hashlib
from pathlib import Path
import unittest

import yaml


ROOT = Path(__file__).parents[1]
POLICY_SHA256 = "0e810ef485df26b1886f2477c65d568905c741deeaf864b63403ef411414ae79"
CHECKPOINT_SHA256 = "8cc7183114802f04726d99faedb39c19928d2f96e1de275bab5b8d2cc98a9331"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


class DeploymentContractTests(unittest.TestCase):
    def test_default_and_archived_policy_are_model_9200_export(self):
        default_policy = ROOT / "models/policy.pt"
        archived_policy = ROOT / "models/policy_adaboot_ideal_l2c2_9200_20260929.pt"
        checkpoint = ROOT / "models/model_adaboot_ideal_l2c2_9200_20260929.pt"

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
                self.assertEqual(controller[f"{mode}_kd_{joint}"], 0.8)

        for node in hardware_nodes:
            hardware = config[node]["ros__parameters"]
            for prefix in ("default_coxa", "default", "default_knee"):
                self.assertEqual(hardware[f"{prefix}_kp"], 40.0)
                self.assertEqual(hardware[f"{prefix}_kd"], 0.8)

        self.assertEqual(controller["safe_park_kp"], 14.0)
        self.assertEqual(controller["safe_park_kd"], 0.5)


if __name__ == "__main__":
    unittest.main()
