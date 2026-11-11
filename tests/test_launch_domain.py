"""Check launch overrides without launching nodes or opening CAN."""
import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch_ros.utilities import evaluate_parameters, normalize_parameters


spec = importlib.util.spec_from_file_location(
    "sim2real_launch", Path(__file__).parents[1] / "launch/sim2real.launch.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class LaunchDomainTests(unittest.TestCase):
    def setUp(self):
        # Domain substitution needs no log files or ROS runtime side effects.
        logger = patch("launch.logging.get_logger")
        logger.start()
        self.addCleanup(logger.stop)

    def domain(self, exported=None, override=None):
        context = LaunchContext()
        context.environment.pop("ROS_DOMAIN_ID", None)
        if exported is not None:
            context.environment["ROS_DOMAIN_ID"] = exported
        if override is not None:
            context.launch_configurations["domain_id"] = override
        actions = module.generate_launch_description().entities
        for action in actions:
            if isinstance(action, DeclareLaunchArgument) and action.name == "domain_id":
                action.execute(context)
            elif isinstance(action, SetEnvironmentVariable):
                action.execute(context)
                return context.environment["ROS_DOMAIN_ID"]
        self.fail("Domain must be applied before launching nodes")

    def test_unset_domain_is_isolated(self):
        self.assertEqual(self.domain(), "1")

    def test_exported_domain_is_preserved(self):
        self.assertEqual(self.domain(exported="17"), "17")

    def test_launch_override_wins(self):
        self.assertEqual(self.domain(exported="17", override="18"), "18")


class LaunchPolicyTests(unittest.TestCase):
    def zero_controller_parameters(self, **overrides):
        root = Path(__file__).parents[1]
        zero_spec = importlib.util.spec_from_file_location("zero_launch", root / "launch/sim2real_zero.launch.py")
        zero_module = importlib.util.module_from_spec(zero_spec)
        zero_spec.loader.exec_module(zero_module)
        context = LaunchContext()
        context.launch_configurations.update(with_hardware="false", domain_id="17")
        context.launch_configurations.update(overrides)
        with patch("launch.logging.get_logger"):
            for action in zero_module.generate_launch_description().entities:
                if isinstance(action, DeclareLaunchArgument):
                    action.execute(context)
            actions = module.generate_launch_description().entities
            for action in actions:
                if isinstance(action, DeclareLaunchArgument):
                    action.execute(context)
            with patch.object(module, "Node") as node:
                module._launch_controller(context)
        parameters = evaluate_parameters(context, normalize_parameters(node.call_args.kwargs["parameters"]))
        return context, parameters

    def test_zero_launcher_passes_matching_policy_and_reference_to_controller(self):
        root = Path(__file__).parents[1]
        context, parameters = self.zero_controller_parameters()
        self.assertEqual(parameters[1]["policy_reference"], "zero")
        self.assertEqual(parameters[1]["policy_path"],
                         str(root / "models/policy_adaboot_ideal_l2c2_zero_reference_5999_20261103.pt"))
        self.assertEqual(context.launch_configurations["with_hardware"], "false")
        self.assertEqual(context.launch_configurations["domain_id"], "17")

    def test_zero_launcher_preserves_explicit_model_and_config_overrides(self):
        context, parameters = self.zero_controller_parameters(
            policy_path="/tmp/custom_zero.pt", config_file="/tmp/custom.yaml",
            domain_id="23")
        self.assertEqual(parameters[0], Path("/tmp/custom.yaml"))
        self.assertEqual(parameters[1], {
            "policy_path": "/tmp/custom_zero.pt", "policy_reference": "zero",
        })
        self.assertEqual(context.launch_configurations["domain_id"], "23")

    def test_zero_launcher_rejects_nominal_reference(self):
        with self.assertRaises(RuntimeError):
            self.zero_controller_parameters(policy_reference="nominal")


if __name__ == "__main__":
    unittest.main()
