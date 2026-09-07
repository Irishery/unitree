#!/usr/bin/env python3
"""Request one global path from the physical G1 planning-only Nav2 stack."""

import argparse
import math
import sys

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.node import Node


def parse_args():
    parser = argparse.ArgumentParser(
        description="Build a path on /map without commanding the physical G1."
    )
    parser.add_argument("x", type=float, help="goal X in map frame, metres")
    parser.add_argument("y", type=float, help="goal Y in map frame, metres")
    parser.add_argument("yaw", type=float, nargs="?", default=0.0, help="goal yaw, radians")
    args = parser.parse_args()
    if not all(math.isfinite(value) for value in (args.x, args.y, args.yaw)):
        parser.error("X, Y, and YAW must be finite numbers")
    return args


def main():
    args = parse_args()
    rclpy.init()
    node = Node("g1_hardware_path_request")
    client = ActionClient(node, ComputePathToPose, "/compute_path_to_pose")

    try:
        print("Waiting up to 15 s for planning-only Nav2...")
        if not client.wait_for_server(timeout_sec=15.0):
            print("ERROR: /compute_path_to_pose is unavailable", file=sys.stderr)
            return 1

        goal = ComputePathToPose.Goal()
        goal.goal.header.frame_id = "map"
        goal.goal.header.stamp = node.get_clock().now().to_msg()
        goal.goal.pose.position.x = args.x
        goal.goal.pose.position.y = args.y
        goal.goal.pose.orientation.z = math.sin(args.yaw * 0.5)
        goal.goal.pose.orientation.w = math.cos(args.yaw * 0.5)
        goal.planner_id = "GridBased"
        goal.use_start = False

        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, send_future, timeout_sec=10.0)
        if not send_future.done() or send_future.result() is None:
            print("ERROR: planner did not acknowledge the request", file=sys.stderr)
            return 1
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            print("ERROR: planner rejected the request", file=sys.stderr)
            return 1

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=30.0)
        if not result_future.done() or result_future.result() is None:
            goal_handle.cancel_goal_async()
            print("ERROR: path planning timed out", file=sys.stderr)
            return 1

        wrapped_result = result_future.result()
        path = wrapped_result.result.path
        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED or not path.poses:
            print(
                f"ERROR: no path; action status={wrapped_result.status}",
                file=sys.stderr,
            )
            return 1

        length = sum(
            math.hypot(
                second.pose.position.x - first.pose.position.x,
                second.pose.position.y - first.pose.position.y,
            )
            for first, second in zip(path.poses, path.poses[1:])
        )
        start = path.poses[0].pose.position
        end = path.poses[-1].pose.position
        print(
            "Path built: "
            f"poses={len(path.poses)}, length={length:.2f} m, "
            f"start=({start.x:.2f}, {start.y:.2f}), "
            f"goal=({end.x:.2f}, {end.y:.2f})"
        )
        print("This command only planned the path; it did not publish any velocity.")
        return 0
    finally:
        client.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
