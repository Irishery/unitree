#!/usr/bin/env python3
"""Turn RViz /goal_pose clicks into planning-only ComputePathToPose goals."""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class PlannerGoalBridge(Node):
    def __init__(self):
        super().__init__("g1_planner_goal_bridge")
        self._client = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")
        self._goal_pending = False
        self._subscription = self.create_subscription(
            PoseStamped, "/goal_pose", self._goal_callback, 1
        )
        self.get_logger().info(
            "Planning-only RViz goal bridge ready on /goal_pose; it publishes no velocity"
        )

    def _goal_callback(self, pose):
        if self._goal_pending:
            self.get_logger().warning("A path request is already running; ignoring this click")
            return
        if not self._client.server_is_ready():
            self.get_logger().error("/compute_path_to_pose is not ready")
            return
        if not pose.header.frame_id:
            self.get_logger().error("RViz goal has an empty frame_id")
            return

        request = ComputePathToPose.Goal()
        request.goal = pose
        request.planner_id = "GridBased"
        request.use_start = False
        self._goal_pending = True
        future = self._client.send_goal_async(request)
        future.add_done_callback(self._goal_response)

    def _goal_response(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self._goal_pending = False
            self.get_logger().error("Planner rejected the RViz path request")
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._path_result)

    def _path_result(self, future):
        self._goal_pending = False
        wrapped_result = future.result()
        if wrapped_result is None or not wrapped_result.result.path.poses:
            self.get_logger().error("Planner returned no path")
            return
        poses = wrapped_result.result.path.poses
        length = sum(
            math.hypot(
                second.pose.position.x - first.pose.position.x,
                second.pose.position.y - first.pose.position.y,
            )
            for first, second in zip(poses, poses[1:])
        )
        self.get_logger().info(
            f"Path built: {len(poses)} poses, approximately {length:.2f} m; no motion sent"
        )


def main():
    rclpy.init()
    node = PlannerGoalBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
