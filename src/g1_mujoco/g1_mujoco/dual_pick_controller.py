"""ROS executor adapter for the perception-driven bimanual pick sequence."""
import math
import os
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String

from .bimanual_sequence import SmoothSequence, build_pick_plan
from .grasp_planning import (
    ARM_JOINTS, BASE_POSE, FINGER_OPEN, GraspKinematics, GraspParams,
    HAND_JOINTS)


def _normalise_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


# Hardcoded dead-reckoning waypoints (odom frame) for the carry from the main
# table to the side table: back away from the main table, turn right 90
# degrees, then drive forward and strafe until the held box is over the side
# table.  No lidar or Nav2 is used; the mobile base is driven kinematically
# from /odom feedback only.
TRANSPORT_WAYPOINTS = (
    (-0.25, 0.0, 0.0),
    (-0.25, 0.0, -math.pi / 2.0),
    (-0.25, -0.42, -math.pi / 2.0),
)
TRANSPORT_POSITION_TOL = 0.02
TRANSPORT_YAW_TOL = 0.06
TRANSPORT_WAYPOINT_TIMEOUT = 90.0
TRANSPORT_MAX_LINEAR = 0.16
TRANSPORT_MAX_ANGULAR = 0.45
TRANSPORT_LINEAR_ACCEL = 0.3
TRANSPORT_ANGULAR_ACCEL = 0.8


class DualPickController(Node):
    """Own scenario state while delegating physics/hardware I/O to ROS topics."""

    def __init__(self):
        super().__init__("g1_dual_pick_controller")
        self.declare_parameter("pose_timeout", 0.75)
        self.declare_parameter("model_z_offset", float(BASE_POSE[2]))
        self.declare_parameter("box_side_rails", False)
        self.declare_parameter("box_length", 0.150)
        self.declare_parameter("box_width", 0.250)
        self.declare_parameter("box_height", 0.140)
        self.declare_parameter("transport", False)
        description = Path(os.environ.get(
            "G1_DESCRIPTION_DIR", "/opt/unitree_ros/robots/g1_description"))
        scene = ("g1_29dof_with_dex3_tabletop_rails.xml"
                 if bool(self.get_parameter("box_side_rails").value)
                 else "g1_29dof_with_dex3_tabletop.xml")
        self.kinematics = GraspKinematics(description / scene)
        self.grasp_params = GraspParams(box_dims=tuple(float(
            self.get_parameter(name).value) for name in (
                "box_length", "box_width", "box_height")))
        self.pose = None
        self.pose_received = None
        self.joints = {}
        self.sequence = None
        self.failed = None
        self.odom = None
        self.transport_index = 0
        self.transport_active = False
        self.transport_deadline = 0.0
        self.arm_pubs = {name: self.create_publisher(
            Float64, f"/g1/mujoco/joints/{name}/command", 10)
            for names in ARM_JOINTS.values() for name in names}
        self.hand_pubs = {side: self.create_publisher(
            JointState, f"/g1/dex3/{side}/command", 10) for side in ARM_JOINTS}
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.camera_pub = self.create_publisher(Bool, "/g1/mujoco/camera_enable", 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.status_pub = self.create_publisher(String, "/g1/pick/status", 10)
        self.create_subscription(Bool, "/g1/pick/start", self.on_start, 10)
        self.create_subscription(Bool, "/g1/pick/cancel", self.on_cancel, 10)
        self.create_subscription(PoseStamped, "/g1/perception/box_pose", self.on_pose, 10)
        self.create_subscription(JointState, "/g1/joint_states", self.on_joints, 10)
        self.create_timer(0.02, self.tick)

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_pose(self, message):
        self.pose, self.pose_received = message, self.now_seconds()

    def on_joints(self, message):
        self.joints.update(zip(message.name, message.position))

    def on_odom(self, message):
        pose = message.pose.pose
        q = pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.odom = (pose.position.x, pose.position.y, yaw)

    def on_cancel(self, message):
        if message.data:
            self.sequence, self.failed = None, "cancelled"

    def on_start(self, message):
        if not message.data or self.sequence is not None:
            return
        now = self.now_seconds()
        if self.pose is None or self.pose_received is None or now - self.pose_received > float(
                self.get_parameter("pose_timeout").value):
            self.failed = "no_fresh_rgbd_pose"
            return
        if any(name not in self.joints for side in ARM_JOINTS for name in ARM_JOINTS[side]):
            self.failed = "no_joint_state"
            return
        p, q = self.pose.pose.position, self.pose.pose.orientation
        centre = np.array([p.x, p.y, p.z], dtype=float)
        if self.pose.header.frame_id == "pelvis":
            centre[2] += float(self.get_parameter("model_z_offset").value)
        yaw = math.atan2(2.0 * (q.w*q.z + q.x*q.y), 1.0 - 2.0 * (q.y*q.y + q.z*q.z))
        transport = bool(self.get_parameter("transport").value)
        try:
            plan = build_pick_plan(
                self.kinematics, centre, yaw, params=self.grasp_params,
                transport=transport)
        except RuntimeError as error:
            self.failed = f"planning_failed:{error}"
            self.get_logger().error(self.failed)
            return
        initial_arms = {s: np.array([self.joints[n] for n in ARM_JOINTS[s]]) for s in ARM_JOINTS}
        initial_hands = {s: np.array([self.joints.get(n, FINGER_OPEN[s][i])
                                      for i, n in enumerate(HAND_JOINTS[s])]) for s in ARM_JOINTS}
        self.sequence = SmoothSequence(plan, initial_arms, initial_hands)
        self.sequence.started = now
        self.transport_index = 0
        self.transport_active = False
        self.transport_deadline = now + TRANSPORT_WAYPOINT_TIMEOUT
        self.failed = None
        self.camera_pub.publish(Bool(data=False))
        self.get_logger().info(f"planned bimanual pick from RGB-D: centre={centre}, yaw={yaw:.3f}")

    def publish_commands(self, arms, hands):
        for side in ARM_JOINTS:
            for name, value in zip(ARM_JOINTS[side], arms[side]):
                self.arm_pubs[name].publish(Float64(data=float(value)))
            message = JointState()
            message.name = HAND_JOINTS[side]
            message.position = [float(v) for v in hands[side]]
            self.hand_pubs[side].publish(message)

    def _stop_base(self):
        self.cmd_vel_pub.publish(Twist())

    def drive_transport(self, now):
        """Dead-reckon the mobile base while both hands hold the box."""
        segment = self.sequence.current_segment
        if segment is None or not segment.transport:
            return
        if self.odom is None:
            self.status_pub.publish(String(data="transport_waiting_odom"))
            return
        if not self.transport_active:
            self.transport_active = True
            self.transport_index = 0
            self.transport_deadline = now + TRANSPORT_WAYPOINT_TIMEOUT
            self.transport_last_cmd = (0.0, 0.0, 0.0)
            self.get_logger().info("base transport started")
        if self.transport_index >= len(TRANSPORT_WAYPOINTS):
            self._stop_base()
            self.transport_active = False
            self.sequence.advance(now)
            self.get_logger().info("base transport finished; resuming place")
            return
        target_x, target_y, target_yaw = TRANSPORT_WAYPOINTS[self.transport_index]
        odom_x, odom_y, odom_yaw = self.odom
        error_x = target_x - odom_x
        error_y = target_y - odom_y
        yaw_error = _normalise_angle(target_yaw - odom_yaw)
        if (math.hypot(error_x, error_y) < TRANSPORT_POSITION_TOL
                and abs(yaw_error) < TRANSPORT_YAW_TOL):
            self.transport_index += 1
            self.transport_deadline = now + TRANSPORT_WAYPOINT_TIMEOUT
            self.transport_last_cmd = (0.0, 0.0, 0.0)
            self._stop_base()
            return
        if now > self.transport_deadline:
            self.get_logger().warn(
                f"transport waypoint {self.transport_index} timeout; skipping")
            self.transport_index += 1
            self.transport_deadline = now + TRANSPORT_WAYPOINT_TIMEOUT
            self.transport_last_cmd = (0.0, 0.0, 0.0)
            self._stop_base()
            return
        world_vx = max(-TRANSPORT_MAX_LINEAR, min(TRANSPORT_MAX_LINEAR, 1.2 * error_x))
        world_vy = max(-TRANSPORT_MAX_LINEAR, min(TRANSPORT_MAX_LINEAR, 1.2 * error_y))
        cos_yaw, sin_yaw = math.cos(odom_yaw), math.sin(odom_yaw)
        desired = (
            cos_yaw * world_vx + sin_yaw * world_vy,
            -sin_yaw * world_vx + cos_yaw * world_vy,
            max(-TRANSPORT_MAX_ANGULAR, min(TRANSPORT_MAX_ANGULAR, 1.5 * yaw_error)),
        )
        # Slew-limit the setpoint so the kinematic base does not jerk the box
        # out of the compliant DEX3 grip.
        previous = self.transport_last_cmd
        line_step = TRANSPORT_LINEAR_ACCEL * 0.02
        yaw_step = TRANSPORT_ANGULAR_ACCEL * 0.02
        command = Twist()
        command.linear.x = previous[0] + max(-line_step, min(line_step, desired[0] - previous[0]))
        command.linear.y = previous[1] + max(-line_step, min(line_step, desired[1] - previous[1]))
        command.angular.z = previous[2] + max(-yaw_step, min(yaw_step, desired[2] - previous[2]))
        self.transport_last_cmd = (command.linear.x, command.linear.y, command.angular.z)
        self.cmd_vel_pub.publish(command)

    def tick(self):
        if self.sequence is None:
            self.status_pub.publish(String(data=f"failed:{self.failed}" if self.failed else "idle"))
            return
        now = self.now_seconds()
        segment = self.sequence.current_segment
        stage = self.sequence.stage
        arms, hands = self.sequence.sample(now)
        self.publish_commands(arms, hands)  # continuous even during settle/hold
        if segment is not None and segment.transport:
            self.drive_transport(now)
        self.status_pub.publish(String(data=stage))
        if self.sequence.done:
            self._stop_base()
            self.camera_pub.publish(Bool(data=True))
            self.get_logger().info("bimanual pick/place sequence completed")
            self.sequence = None


def main():
    rclpy.init()
    node = DualPickController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
