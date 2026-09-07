"""No ROS or robot required: exercise cleanup outcomes with mocked transport."""
import importlib.util
import itertools
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import MagicMock, patch


def load_probe():
    modules = {name: types.ModuleType(name) for name in (
        "rclpy", "rclpy.node", "rclpy.qos", "rclpy.signals",
        "geometry_msgs", "geometry_msgs.msg", "std_msgs", "std_msgs.msg",
        "std_srvs", "std_srvs.srv",
    )}
    modules["rclpy.node"].Node = object
    for name in ("DurabilityPolicy", "QoSProfile", "ReliabilityPolicy"):
        setattr(modules["rclpy.qos"], name, MagicMock())
    modules["rclpy.signals"].SignalHandlerOptions = MagicMock()
    modules["geometry_msgs.msg"].Twist = MagicMock
    modules["std_msgs.msg"].Bool = MagicMock
    modules["std_srvs.srv"].SetBool = types.SimpleNamespace(Request=MagicMock)
    spec = importlib.util.spec_from_file_location(
        "motion_probe_under_test", Path(__file__).resolve().parents[1] / "hardware_motion_probe.py"
    )
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module


probe = load_probe()


class CleanupTests(unittest.TestCase):
    def setUp(self):
        self.node = probe.MotionProbe.__new__(probe.MotionProbe)
        self.node.control_enabled = False  # A cached false must not suffice.
        self.node.publish_for = MagicMock()
        self.node.disable_client = MagicMock()
        self.future = self.node.disable_client.call_async.return_value
        self.future.done.return_value = True
        self.future.result.return_value = types.SimpleNamespace(success=True)
        probe.rclpy.spin_until_future_complete = MagicMock()
        probe.rclpy.spin_once = MagicMock()
        self.clock = patch.object(probe.time, "monotonic", side_effect=itertools.count(0, 0.25))
        self.clock.start()
        self.addCleanup(self.clock.stop)

    def confirm_false(self, *_args, **_kwargs):
        self.node.control_enabled = False

    def test_success_requires_service_and_fresh_state(self):
        probe.rclpy.spin_once.side_effect = self.confirm_false
        self.assertTrue(self.node.stop_and_disarm())
        self.node.publish_for.assert_called_once_with(0.0, 0.5)
        self.assertFalse(self.node.disable_client.call_async.call_args.args[0].data)

    def test_missing_service(self):
        self.node.disable_client.wait_for_service.return_value = False
        self.assertFalse(self.node.stop_and_disarm())

    def test_service_timeout(self):
        self.future.done.return_value = False
        self.assertFalse(self.node.stop_and_disarm())
        self.future.result.assert_not_called()

    def test_service_rejection(self):
        self.future.result.return_value = types.SimpleNamespace(success=False, message="denied")
        self.assertFalse(self.node.stop_and_disarm())

    def test_service_exception(self):
        self.future.result.side_effect = RuntimeError("transport failure")
        self.assertFalse(self.node.stop_and_disarm())

    def test_stale_false_is_not_confirmation(self):
        self.assertFalse(self.node.stop_and_disarm())

    def test_stop_publication_failure_still_attempts_disarm(self):
        self.node.publish_for.side_effect = RuntimeError("publish failed")
        probe.rclpy.spin_once.side_effect = self.confirm_false
        self.assertTrue(self.node.stop_and_disarm())
        self.node.disable_client.call_async.assert_called_once()

    def test_lost_arm_prevents_nonzero_publication(self):
        self.node.publisher = MagicMock()
        with self.assertRaises(RuntimeError):
            probe.MotionProbe.publish_for(self.node, 0.05, 0.5)
        self.node.publisher.publish.assert_not_called()


class ExitStatusTests(unittest.TestCase):
    def run_main(self, disarm_success, publish_error=None, args=None):
        node = MagicMock()
        node.control_enabled = True
        node.count_subscribers.return_value = 1
        node.stop_and_disarm.return_value = disarm_success
        node.publish_for.side_effect = publish_error
        with patch.object(probe, "MotionProbe", return_value=node), \
                patch.object(probe, "rclpy", MagicMock()), \
                patch.object(probe.signal, "signal"), \
                patch.object(probe.sys, "argv", ["hardware_motion_probe.py"] + (args or [])), \
                patch.object(probe.time, "monotonic", side_effect=itertools.count(0, .5)):
            code = probe.main()
        node.stop_and_disarm.assert_called_once()
        node.destroy_node.assert_called_once()
        expected = {
            (): (0.05, 0.5),
            ("--gait-start",): (0.20, 0.5),
            ("--twenty-cm",): (0.20, 1.0),
        }[tuple(args or [])]
        node.publish_for.assert_called_once_with(*expected)
        return code

    def test_unconfirmed_disarm_is_failure(self):
        self.assertEqual(self.run_main(False), 3)

    def test_success_is_zero(self):
        self.assertEqual(self.run_main(True), 0)

    def test_interrupt_still_disarms(self):
        self.assertEqual(self.run_main(True, KeyboardInterrupt()), 130)

    def test_twenty_cm_keeps_speed_and_bounds_duration(self):
        self.assertEqual(self.run_main(True, args=["--twenty-cm"]), 0)

    def test_gait_start_uses_verified_threshold_and_short_duration(self):
        self.assertEqual(self.run_main(True, args=["--gait-start"]), 0)

    def test_twenty_cm_interrupt_still_disarms(self):
        self.assertEqual(self.run_main(True, KeyboardInterrupt(), ["--twenty-cm"]), 130)

    def test_invalid_options_never_initialize_ros(self):
        for args in (["--twenty-cm", "--twenty-cm"], ["--speed", "1"], ["--duration", "40"]):
            with self.subTest(args=args), \
                    patch.object(probe.sys, "argv", ["probe"] + args), \
                    patch.object(probe, "rclpy", MagicMock()) as ros:
                self.assertEqual(probe.main(), 2)
                ros.init.assert_not_called()


if __name__ == "__main__":
    unittest.main()
