"""Regression tests for the pinned sim2real model and actuator gains."""

import ast
import hashlib
import json
from pathlib import Path
import unittest
from types import SimpleNamespace

import numpy as np
import yaml


ROOT = Path(__file__).parents[1]
POLICY_SHA256 = "15c2e2cf7775361b7edbebe3e2181f0e4bae5ad8024c826b340cffb891bef5a0"
CHECKPOINT_SHA256 = "4544c25e7602a294b8ed6df58e98532c3a1a08b066741b4eab56e386b5e56311"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


class DeploymentContractTests(unittest.TestCase):
    def test_default_policy_and_checkpoint_are_jaguar_29996_export(self):
        default_policy = ROOT / "models/policy.pt"
        checkpoint = ROOT / "models/model_adaboot_ideal_l2c2_kp45kd45_clear8cm_29996_20261022.pt"

        self.assertEqual(_sha256(default_policy), POLICY_SHA256)
        self.assertEqual(sorted(path.name for path in (ROOT / "models").glob("policy*.pt")),
                         ["policy.pt"])
        manifest = json.loads((ROOT / "models/deployment_adaboot_ideal_l2c2_kp45kd45_clear8cm_29996_20261022.json").read_text())
        self.assertEqual(manifest["policy_file"], default_policy.name)
        self.assertEqual(manifest["policy_sha256"], POLICY_SHA256)
        self.assertEqual(manifest["checkpoint_sha256"], CHECKPOINT_SHA256)
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
                self.assertEqual(controller[f"{mode}_kp_{joint}"], 45.0)
                self.assertEqual(controller[f"{mode}_kd_{joint}"], 4.5)

        for node in hardware_nodes:
            hardware = config[node]["ros__parameters"]
            for prefix in ("default_coxa", "default", "default_knee"):
                self.assertEqual(hardware[f"{prefix}_kp"], 45.0)
                self.assertEqual(hardware[f"{prefix}_kd"], 4.5)

        self.assertEqual(controller["safe_park_kp"], 14.0)
        self.assertEqual(controller["safe_park_kd"], 0.5)

    def test_controller_nominal_pose_matches_training(self):
        source = (ROOT / "scripts/nxp_jaguar_controller.py").read_text()
        self.assertIn("-1.51, -1.51, -1.49, -1.49", source)
        self.assertIn("1.22,  1.22,  1.20,  1.20", source)

    def test_standing_pose_and_walk_ramp_preserve_policy_reference(self):
        parameters = ast.parse((ROOT / "scripts/parameters.py").read_text())
        stand = next(node.value for node in parameters.body
                     if isinstance(node, ast.Assign)
                     and any(isinstance(t, ast.Name) and t.id == "STAND_ANGLE"
                             for t in node.targets))
        stand_ros = ast.literal_eval(stand)
        np.testing.assert_array_equal(stand_ros,
                                      [0.0, -1.49, 1.20] * 2 + [0.0, -1.51, 1.22] * 2)

        source = ast.parse((ROOT / "scripts/nxp_jaguar_controller.py").read_text())
        names = {"ROS_TO_ISAAC", "STAND_JOINT_POS", "DEFAULT_JOINT_POS", "ACTION_SCALE"}
        assignments = [node for node in source.body if isinstance(node, ast.Assign)
                       and any(isinstance(t, ast.Name) and t.id in names for t in node.targets)]
        namespace = {"np": np, "P": SimpleNamespace(STAND_ANGLE=stand_ros,
                                                         ROS_TO_ISAAC=[9, 6, 3, 0, 10, 7, 4, 1, 11, 8, 5, 2])}
        exec(compile(ast.Module(body=assignments, type_ignores=[]), "<poses>", "exec"), namespace)
        np.testing.assert_allclose(namespace["STAND_JOINT_POS"],
                                   [0.0] * 4 + [-1.51, -1.51, -1.49, -1.49]
                                   + [1.22, 1.22, 1.20, 1.20], atol=1e-7)

        # Execute the actual WALK target expression at both ramp endpoints.
        target = next(node for node in ast.walk(source)
                      if isinstance(node, ast.Assign)
                      and any(isinstance(t, ast.Name) and t.id == "target_pos" for t in node.targets)
                      and any(isinstance(n, ast.Name) and n.id == "applied_action"
                              for n in ast.walk(node.value)))
        action = np.linspace(-0.5, 0.5, 12)
        for ramp in (0.0, 0.5, 1.0):
            namespace.update(ramp=ramp, applied_action=ramp * action)
            exec(compile(ast.Module(body=[target], type_ignores=[]), "<walk target>", "exec"), namespace)
            expected = ((1.0 - ramp) * namespace["STAND_JOINT_POS"]
                        + ramp * (namespace["DEFAULT_JOINT_POS"] + 0.25 * action))
            np.testing.assert_allclose(namespace["target_pos"], expected, atol=1e-7)


if __name__ == "__main__":
    unittest.main()
