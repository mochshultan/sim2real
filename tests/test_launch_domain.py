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



if __name__ == "__main__":
    unittest.main()
