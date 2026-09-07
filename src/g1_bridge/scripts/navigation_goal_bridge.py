#!/usr/bin/env python3
"""Plan and follow an RViz goal using the guarded physical-G1 Nav2 stack."""

import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, FollowPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger


def path_metrics(path):
    points = [(pose.pose.position.x, pose.pose.position.y) for pose in path.poses]
    length = 0.0
    headings = []
    for first, second in zip(points, points[1:]):
        dx = second[0] - first[0]
        dy = second[1] - first[1]
        segment_length = math.hypot(dx, dy)
        length += segment_length
        if segment_length >= 0.02:
            headings.append(math.atan2(dy, dx))
    orientation = path.poses[0].pose.orientation
    initial_yaw = math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
    )
    heading_change = 0.0
    if headings:
        heading_change += abs(
            math.atan2(
                math.sin(headings[0] - initial_yaw),
                math.cos(headings[0] - initial_yaw),
            )
        )
    heading_change += sum(
        abs(math.atan2(math.sin(second - first), math.cos(second - first)))
        for first, second in zip(headings, headings[1:])
    )
    return length, heading_change


class NavigationGoalBridge(Node):
    def __init__(self):
        super().__init__("g1_navigation_goal_bridge")
        self.declare_parameter("min_path_length", 0.10)
        self.declare_parameter("max_path_length", 0.0)
        self.declare_parameter("max_path_heading_change", 0.35)
        self._min_path_length = float(self.get_parameter("min_path_length").value)
        self._max_path_length = float(self.get_parameter("max_path_length").value)
        self._max_path_heading_change = float(
            self.get_parameter("max_path_heading_change").value
        )
        if self._min_path_length < 0.0 or (
            self._max_path_length > 0.0
            and self._min_path_length >= self._max_path_length
        ):
            raise ValueError("invalid trial path length bounds")

        self._planner = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")
        self._controller = ActionClient(self, FollowPath, "/follow_path")
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._state_publisher = self.create_publisher(
            String, "/g1/navigation_state", state_qos
        )
        self._control_subscription = self.create_subscription(
            Bool, "/g1/control_enabled", self._on_control, state_qos
        )
        self._goal_subscription = self.create_subscription(
            PoseStamped, "/goal_pose", self._on_goal, 1
        )
        self._cancel_service = self.create_service(
            Trigger, "/g1/cancel_navigation", self._on_cancel
        )
        self._disarm_client = self.create_client(SetBool, "/g1/enable_control")
        self._control_enabled = False
        self._request_pending = False
        self._follow_goal_handle = None
        self._generation = 0
        self._publish_state("DISARMED")
        length_text = (
            f">= {self._min_path_length:.2f} m"
            if self._max_path_length <= 0.0
            else f"{self._min_path_length:.2f}..{self._max_path_length:.2f} m"
        )
        self.get_logger().warning(
            f"Navigation ready: arm explicitly, then send a nearly straight "
            f"{length_text} /goal_pose"
        )

    def _publish_state(self, state):
        message = String()
        message.data = state
        self._state_publisher.publish(message)
        self.get_logger().info(f"Navigation state: {state}")

    def _on_control(self, message):
        enabled = bool(message.data)
        if self._control_enabled and not enabled:
            self._cancel_active("DISARMED")
        self._control_enabled = enabled
        if enabled and not self._request_pending and self._follow_goal_handle is None:
            self._publish_state("ARMED_WAITING_FOR_GOAL")

    def _on_goal(self, pose):
        if not self._control_enabled:
            self._publish_state("REJECTED_DISARMED")
            return
        if self._request_pending or self._follow_goal_handle is not None:
            self._publish_state("REJECTED_BUSY")
            return
        if not pose.header.frame_id:
            self._publish_state("REJECTED_EMPTY_FRAME")
            return
        pose_values = (
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        )
        if not all(math.isfinite(value) for value in pose_values):
            self._publish_state("REJECTED_NONFINITE_GOAL")
            self._request_disarm()
            return
        if not self._planner.server_is_ready() or not self._controller.server_is_ready():
            self._publish_state("REJECTED_NAV2_NOT_READY")
            self._request_disarm()
            return

        request = ComputePathToPose.Goal()
        request.goal = pose
        request.planner_id = "GridBased"
        request.use_start = False
        self._generation += 1
        generation = self._generation
        self._request_pending = True
        self._publish_state("PLANNING")
        self._planner.send_goal_async(request).add_done_callback(
            lambda future: self._plan_response(future, generation)
        )

    def _plan_response(self, future, generation):
        if generation != self._generation:
            return
        try:
            goal_handle = future.result()
        except Exception as error:
            self._request_pending = False
            self._publish_state(f"PLAN_RESPONSE_ERROR_{type(error).__name__}")
            self._request_disarm()
            return
        if goal_handle is None or not goal_handle.accepted:
            self._request_pending = False
            self._publish_state("PLAN_REJECTED")
            self._request_disarm()
            return
        goal_handle.get_result_async().add_done_callback(
            lambda result_future: self._plan_result(result_future, generation)
        )

    def _plan_result(self, future, generation):
        if generation != self._generation:
            return
        try:
            wrapped = future.result()
        except Exception as error:
            self._request_pending = False
            self._publish_state(f"PLAN_RESULT_ERROR_{type(error).__name__}")
            self._request_disarm()
            return
        if (
            wrapped is None
            or wrapped.status != GoalStatus.STATUS_SUCCEEDED
            or not wrapped.result.path.poses
        ):
            self._request_pending = False
            self._publish_state("PLAN_FAILED")
            self._request_disarm()
            return
        length, heading_change = path_metrics(wrapped.result.path)
        if not math.isfinite(length) or not math.isfinite(heading_change):
            self._request_pending = False
            self._publish_state("REJECTED_NONFINITE_PATH")
            self._request_disarm()
            return
        if length < self._min_path_length or (
            self._max_path_length > 0.0 and length > self._max_path_length
        ):
            self._request_pending = False
            self._publish_state(
                f"REJECTED_PATH_LENGTH_{length:.2f}M"
            )
            self._request_disarm()
            return
        if heading_change > self._max_path_heading_change:
            self._request_pending = False
            self._publish_state(
                f"REJECTED_PATH_TURN_{heading_change:.2f}RAD"
            )
            self._request_disarm()
            return
        if not self._control_enabled:
            self._request_pending = False
            self._publish_state("DISARMED_BEFORE_FOLLOW")
            return

        follow = FollowPath.Goal()
        follow.path = wrapped.result.path
        follow.controller_id = "FollowPath"
        self._publish_state(
            f"FOLLOW_REQUEST_{length:.2f}M_TURN_{heading_change:.2f}RAD"
        )
        self._controller.send_goal_async(follow).add_done_callback(
            lambda future: self._follow_response(future, generation)
        )

    def _follow_response(self, future, generation):
        if generation != self._generation:
            return
        self._request_pending = False
        try:
            goal_handle = future.result()
        except Exception as error:
            self._publish_state(f"FOLLOW_RESPONSE_ERROR_{type(error).__name__}")
            self._request_disarm()
            return
        if goal_handle is None or not goal_handle.accepted:
            self._publish_state("FOLLOW_REJECTED")
            self._request_disarm()
            return
        if not self._control_enabled:
            goal_handle.cancel_goal_async()
            self._publish_state("DISARMED_BEFORE_FOLLOW")
            return
        self._follow_goal_handle = goal_handle
        self._publish_state("FOLLOWING")
        goal_handle.get_result_async().add_done_callback(
            lambda result_future: self._follow_result(result_future, generation)
        )

    def _follow_result(self, future, generation):
        if generation != self._generation:
            return
        self._follow_goal_handle = None
        try:
            wrapped = future.result()
        except Exception as error:
            self._publish_state(f"FOLLOW_RESULT_ERROR_{type(error).__name__}_DISARMING")
            self._request_disarm()
            return
        if wrapped is not None and wrapped.status == GoalStatus.STATUS_SUCCEEDED:
            self._publish_state("SUCCEEDED_DISARMING")
        elif wrapped is not None and wrapped.status == GoalStatus.STATUS_CANCELED:
            self._publish_state("CANCELED_DISARMING")
        else:
            self._publish_state("FOLLOW_FAILED_DISARMING")
        self._request_disarm()

    def _cancel_active(self, state="CANCEL_REQUESTED"):
        self._generation += 1
        self._request_pending = False
        if self._follow_goal_handle is not None:
            self._follow_goal_handle.cancel_goal_async()
            self._follow_goal_handle = None
        self._publish_state(state)

    def _on_cancel(self, _request, response):
        self._cancel_active()
        self._request_disarm()
        response.success = True
        response.message = "Navigation cancel and software disarm requested"
        return response

    def _request_disarm(self):
        if not self._disarm_client.service_is_ready():
            self._publish_state("DISARM_SERVICE_UNAVAILABLE_USE_REMOTE")
            return
        request = SetBool.Request()
        request.data = False
        self._disarm_client.call_async(request).add_done_callback(self._disarm_response)

    def _disarm_response(self, future):
        try:
            response = future.result()
        except Exception as error:
            self._publish_state(f"DISARM_ERROR_{type(error).__name__}_USE_REMOTE")
            return
        if response is None or not response.success:
            self._publish_state("DISARM_REJECTED_USE_REMOTE")


def main():
    rclpy.init()
    node = NavigationGoalBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
