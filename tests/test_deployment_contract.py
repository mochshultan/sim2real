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
POLICY_SHA256 = "bc37ca24b38f2d659cfcccb9d1101219cdc5dc576003ebcd2dd5a4049d48ddcb"
CHECKPOINT_SHA256 = "e83782ea5ad0da54add29d2403324adcd8569f5c074d21e43fc87f62b8679913"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


class DeploymentContractTests(unittest.TestCase):
    def test_default_and_identity_policy_match_jaguar_39996_checkpoint(self):
        default_policy = ROOT / "models/policy.pt"
        checkpoint = ROOT / "models/model_adaboot_l2c2_nominal32v_clearance5cm_39996_20261112.pt"

        identity_policy = ROOT / "models" / "policy_adaboot_l2c2_nominal32v_clearance5cm_39996_20261112.pt"

        self.assertEqual(_sha256(default_policy), POLICY_SHA256)
        self.assertEqual(_sha256(identity_policy), POLICY_SHA256)
        manifest = json.loads((ROOT / "models/deployment_adaboot_l2c2_nominal32v_clearance5cm_39996_20261112.json").read_text())
        self.assertEqual(manifest["policy_file"], default_policy.name)
        self.assertEqual(manifest["identity_policy_file"], identity_policy.name)
        self.assertEqual(manifest["policy_sha256"], POLICY_SHA256)
        self.assertEqual(manifest["checkpoint_sha256"], CHECKPOINT_SHA256)
        self.assertEqual(_sha256(checkpoint), CHECKPOINT_SHA256)
        self.assertEqual(manifest["policy_reference"], "nominal")
        self.assertEqual(manifest["clip_actions"], 10.0)

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
                                      [0.0, -1.74, 1.28] + [0.0, -1.59, 1.23]
                                      + [0.0, -1.39, 1.25] * 2)

        source = ast.parse((ROOT / "scripts/nxp_jaguar_controller.py").read_text())
        names = {"ROS_TO_ISAAC", "STAND_JOINT_POS", "DEFAULT_JOINT_POS", "ACTION_SCALE"}
        assignments = [node for node in source.body if isinstance(node, ast.Assign)
                       and any(isinstance(t, ast.Name) and t.id in names for t in node.targets)]
        namespace = {"np": np, "P": SimpleNamespace(STAND_ANGLE=stand_ros,
                                                         ROS_TO_ISAAC=[9, 6, 3, 0, 10, 7, 4, 1, 11, 8, 5, 2])}
        exec(compile(ast.Module(body=assignments, type_ignores=[]), "<poses>", "exec"), namespace)
        np.testing.assert_allclose(namespace["STAND_JOINT_POS"],
                                   [0.0] * 4 + [-1.39, -1.39, -1.59, -1.74]
                                   + [1.25, 1.25, 1.23, 1.28], atol=1e-7)

        # Execute the actual WALK target expression at both ramp endpoints.
        target = next(node for node in ast.walk(source)
                      if isinstance(node, ast.Assign)
                      and any(isinstance(t, ast.Name) and t.id == "target_pos" for t in node.targets)
                      and any(isinstance(n, ast.Name) and n.id == "policy_target"
                              for n in ast.walk(node.value)))
        namespace["self"] = SimpleNamespace(stand_joint_pos=namespace["STAND_JOINT_POS"])
        action = np.linspace(-0.5, 0.5, 12)
        for ramp in (0.0, 0.5, 1.0):
            namespace.update(ramp=ramp, policy_target=namespace["DEFAULT_JOINT_POS"] + 0.25 * action)
            exec(compile(ast.Module(body=[target], type_ignores=[]), "<walk target>", "exec"), namespace)
            expected = ((1.0 - ramp) * namespace["STAND_JOINT_POS"]
                        + ramp * (namespace["DEFAULT_JOINT_POS"] + 0.25 * action))
            np.testing.assert_allclose(namespace["target_pos"], expected, atol=1e-7)


if __name__ == "__main__":
    unittest.main()
