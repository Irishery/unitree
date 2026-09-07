"""Pure safety-policy tests; ROS and Nav2 are mocked."""

import importlib.util
import math
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import MagicMock, patch


SCRIPTS = Path(__file__).resolve().parents[2] / "src" / "g1_bridge" / "scripts"


def _module(name):
    return types.ModuleType(name)


def load_velocity_guard():
    modules = {
        name: _module(name)
        for name in (
            "rclpy",
            "rclpy.node",
            "rclpy.qos",
            "geometry_msgs",
            "geometry_msgs.msg",
            "std_msgs",
            "std_msgs.msg",
        )
    }
    modules["rclpy.node"].Node = object
    for name in ("DurabilityPolicy", "QoSProfile", "ReliabilityPolicy"):
        setattr(modules["rclpy.qos"], name, MagicMock())
    modules["geometry_msgs.msg"].Twist = MagicMock
    modules["std_msgs.msg"].Bool = MagicMock
    spec = importlib.util.spec_from_file_location(
        "nav_velocity_guard_under_test", SCRIPTS / "nav_velocity_guard.py"
    )
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module


def load_goal_bridge():
    modules = {
        name: _module(name)
        for name in (
            "rclpy",
            "rclpy.action",
            "rclpy.node",
            "rclpy.qos",
            "action_msgs",
            "action_msgs.msg",
            "geometry_msgs",
            "geometry_msgs.msg",
            "nav2_msgs",
            "nav2_msgs.action",
            "std_msgs",
            "std_msgs.msg",
            "std_srvs",
            "std_srvs.srv",
        )
    }
    modules["rclpy.action"].ActionClient = MagicMock
    modules["rclpy.node"].Node = object
    for name in ("DurabilityPolicy", "QoSProfile", "ReliabilityPolicy"):
        setattr(modules["rclpy.qos"], name, MagicMock())
    modules["action_msgs.msg"].GoalStatus = types.SimpleNamespace(
        STATUS_SUCCEEDED=4, STATUS_CANCELED=5
    )
    modules["geometry_msgs.msg"].PoseStamped = MagicMock
    modules["nav2_msgs.action"].ComputePathToPose = MagicMock
    modules["nav2_msgs.action"].FollowPath = MagicMock
    modules["std_msgs.msg"].Bool = MagicMock
    modules["std_msgs.msg"].String = MagicMock
    modules["std_srvs.srv"].SetBool = types.SimpleNamespace(Request=MagicMock)
    modules["std_srvs.srv"].Trigger = MagicMock
    spec = importlib.util.spec_from_file_location(
        "navigation_goal_bridge_under_test", SCRIPTS / "navigation_goal_bridge.py"
    )
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module


velocity_guard = load_velocity_guard()
goal_bridge = load_goal_bridge()


class VelocityGuardTests(unittest.TestCase):
    def test_forward_is_promoted_to_verified_gait_speed(self):
        command, reason = velocity_guard.guard_command(0.03, 0.0, 0.04, 0.20, 0.10)
        self.assertEqual(command, (0.20, 0.0, 0.04))
        self.assertEqual(reason, "")

    def test_forward_yaw_is_clamped_and_lateral_is_removed(self):
        command, reason = velocity_guard.guard_command(0.20, 0.4, -0.7, 0.20, 0.10)
        self.assertEqual(command, (0.20, 0.0, -0.10))
        self.assertEqual(reason, "")

    def test_reverse_is_stopped(self):
        command, reason = velocity_guard.guard_command(-0.01, 0.0, 0.0, 0.20, 0.10)
        self.assertEqual(command, (0.0, 0.0, 0.0))
        self.assertEqual(reason, "reverse command")

    def test_in_place_rotation_and_sideways_are_stopped(self):
        for values in ((0.0, 0.0, 0.1), (0.0, 0.1, 0.0)):
            with self.subTest(values=values):
                command, reason = velocity_guard.guard_command(*values, 0.20, 0.10)
                self.assertEqual(command, (0.0, 0.0, 0.0))
                self.assertIn("in-place", reason)

    def test_non_finite_input_is_stopped(self):
        command, reason = velocity_guard.guard_command(
            math.nan, 0.0, 0.0, 0.20, 0.10
        )
        self.assertEqual(command, (0.0, 0.0, 0.0))
        self.assertEqual(reason, "non-finite command")


def make_path(points, yaw=0.0):
    poses = []
    for x, y in points:
        poses.append(
            types.SimpleNamespace(
                pose=types.SimpleNamespace(
                    position=types.SimpleNamespace(x=x, y=y),
                    orientation=types.SimpleNamespace(
                        x=0.0,
                        y=0.0,
                        z=math.sin(yaw * 0.5),
                        w=math.cos(yaw * 0.5),
                    ),
                )
            )
        )
    return types.SimpleNamespace(poses=poses)


class PathPolicyTests(unittest.TestCase):
    def test_straight_path_length_and_turn(self):
        length, turn = goal_bridge.path_metrics(make_path([(0, 0), (0.1, 0), (0.3, 0)]))
        self.assertAlmostEqual(length, 0.3)
        self.assertLess(turn, 0.02)

    def test_path_behind_robot_counts_as_large_initial_turn(self):
        _length, turn = goal_bridge.path_metrics(make_path([(0, 0), (-0.2, 0)]))
        self.assertAlmostEqual(turn, math.pi)

    def test_non_finite_path_is_detectable(self):
        length, turn = goal_bridge.path_metrics(make_path([(0, 0), (math.nan, 0)]))
        self.assertFalse(math.isfinite(length) and math.isfinite(turn))

    def test_corner_accumulates_heading_change(self):
        length, turn = goal_bridge.path_metrics(make_path([(0, 0), (0.2, 0), (0.2, 0.2)]))
        self.assertAlmostEqual(length, 0.4)
        self.assertAlmostEqual(turn, math.pi / 2)

    def test_tiny_segments_do_not_create_false_turns(self):
        length, turn = goal_bridge.path_metrics(
            make_path([(0, 0), (0.001, 0.001), (0.2, 0), (0.3, 0)])
        )
        self.assertGreater(length, 0.3)
        self.assertLess(turn, 0.02)


if __name__ == "__main__":
    unittest.main()
