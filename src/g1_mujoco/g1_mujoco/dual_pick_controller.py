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


# Hardcoded dead-reckoning waypoints (odom frame), used in the order of the
# pick/place cycle.  Even cycles carry the box from the main table to the side
# table; odd cycles carry it back.  No lidar or Nav2 is used; the mobile base
# is driven kinematically from /odom feedback only.
TRANSPORT_WAYPOINTS_MAIN_TO_SIDE = (
    (-0.25, 0.0, 0.0),
    (-0.25, 0.0, -math.pi / 2.0),
    (-0.25, -0.42, -math.pi / 2.0),
)
TRANSPORT_WAYPOINTS_SIDE_TO_MAIN = (
    (-0.25, 0.0, -math.pi / 2.0),
    (-0.25, 0.0, 0.0),
    (-0.06, 0.0, 0.0),
)
TRANSPORT_POSITION_TOL = 0.02
TRANSPORT_YAW_TOL = 0.06
TRANSPORT_WAYPOINT_TIMEOUT = 45.0
TRANSPORT_MAX_LINEAR = 0.16
TRANSPORT_MAX_ANGULAR = 0.45
TRANSPORT_LINEAR_ACCEL = 0.3
TRANSPORT_ANGULAR_ACCEL = 0.8
# Standstill delay after entering the transport segment before commanding the
# base, so the carry grip is settled before the gait starts.
TRANSPORT_SETTLE = 2.0
# A box that settles a few centimetres beyond the original grasp after a place
# is outside the arm workspace, so the next pick's IK misses.  While idle the
# controller re-detects it and steps forward until it is back in reach.
PICK_REACH_M = 0.40
APPROACH_SPEED = 0.08


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
        # 0 means repeat forever; N means run N pick/place cycles.
        self.declare_parameter("cycles", 1)
        # When the locomotion policy drives the base, walk much slower so the
        # gait oscillations do not shake the box out of the DEX3 grip.
        self.declare_parameter("walk", False)
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
        self.transport_waypoints = TRANSPORT_WAYPOINTS_MAIN_TO_SIDE
        self.transport_last_cmd = (0.0, 0.0, 0.0)
        self.transport_move_time = 0.0
        self.cycle_index = 0
        self.next_cycle_time = None
        self.camera_republish = 0.0
        self.approach_until = 0.0
        self.approach_cmd = None
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
            self.next_cycle_time = None
            self._stop_base()
            self.camera_pub.publish(Bool(data=True))

    def on_start(self, message):
        if not message.data or self.sequence is not None:
            return
        self.start_cycle(self.now_seconds())

    def start_cycle(self, now):
        if self.pose is None or self.pose_received is None or now - self.pose_received > float(
                self.get_parameter("pose_timeout").value):
            self.failed = "no_fresh_rgbd_pose"
            return False
        if any(name not in self.joints for side in ARM_JOINTS for name in ARM_JOINTS[side]):
            self.failed = "no_joint_state"
            return False
        p, q = self.pose.pose.position, self.pose.pose.orientation
        centre = np.array([p.x, p.y, p.z], dtype=float)
        if self.pose.header.frame_id == "pelvis":
            centre[2] += float(self.get_parameter("model_z_offset").value)
        yaw = math.atan2(2.0 * (q.w*q.z + q.x*q.y), 1.0 - 2.0 * (q.y*q.y + q.z*q.z))
        transport = bool(self.get_parameter("transport").value)
        self.transport_waypoints = (
            TRANSPORT_WAYPOINTS_MAIN_TO_SIDE if self.cycle_index % 2 == 0
            else TRANSPORT_WAYPOINTS_SIDE_TO_MAIN)
        try:
            plan = build_pick_plan(
                self.kinematics, centre, yaw, params=self.grasp_params,
                transport=transport)
        except RuntimeError as error:
            self.failed = f"planning_failed:{error}"
            self.get_logger().error(self.failed)
            return False
        initial_arms = {s: np.array([self.joints[n] for n in ARM_JOINTS[s]]) for s in ARM_JOINTS}
        initial_hands = {s: np.array([self.joints.get(n, FINGER_OPEN[s][i])
                                      for i, n in enumerate(HAND_JOINTS[s])]) for s in ARM_JOINTS}
        self.sequence = SmoothSequence(plan, initial_arms, initial_hands)
        self.sequence.started = now
        self.transport_index = 0
        self.transport_active = False
        self.transport_deadline = now + TRANSPORT_WAYPOINT_TIMEOUT
        self.next_cycle_time = None
        self.failed = None
        self.camera_pub.publish(Bool(data=False))
        self.get_logger().info(
            f"cycle {self.cycle_index + 1}: planned bimanual pick from RGB-D: "
            f"centre={centre}, yaw={yaw:.3f}")
        return True

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
            self.transport_move_time = now + TRANSPORT_SETTLE
            self.get_logger().info("base transport started")
        if now < self.transport_move_time:
            self._stop_base()
            return
        if self.transport_index >= len(self.transport_waypoints):
            self._stop_base()
            self.transport_active = False
            self.sequence.advance(now)
            self.get_logger().info("base transport finished; resuming place")
            return
        target_x, target_y, target_yaw = self.transport_waypoints[self.transport_index]
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
        walking = bool(self.get_parameter("walk").value)
        max_linear = 0.08 if walking else TRANSPORT_MAX_LINEAR
        max_angular = 0.30 if walking else TRANSPORT_MAX_ANGULAR
        world_vx = max(-max_linear, min(max_linear, 1.2 * error_x))
        world_vy = max(-max_linear, min(max_linear, 1.2 * error_y))
        cos_yaw, sin_yaw = math.cos(odom_yaw), math.sin(odom_yaw)
        desired = (
            cos_yaw * world_vx + sin_yaw * world_vy,
            -sin_yaw * world_vx + cos_yaw * world_vy,
            max(-max_angular, min(max_angular, 1.5 * yaw_error)),
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

    def approach_next_box(self, now):
        """Step toward a re-detected box that settled out of arm reach.

        The camera is turned off during the move (rendering throttles the
        physics) and re-enabled afterwards so the detector can confirm the new
        distance before the pick is planned.
        """
        if self.next_cycle_time is None:
            return False
        if now < self.approach_until:
            if self.approach_cmd is not None:
                self.cmd_vel_pub.publish(self.approach_cmd)
            return True
        if self.approach_cmd is not None:
            self._stop_base()
            self.approach_cmd = None
            self.next_cycle_time = now + 1.0
            return True
        if (self.pose is None or self.pose_received is None
                or now - self.pose_received > float(
                    self.get_parameter("pose_timeout").value)):
            return False
        p = self.pose.pose.position
        if math.hypot(p.x, p.y) <= PICK_REACH_M:
            return False
        command = Twist()
        command.linear.x = max(-APPROACH_SPEED, min(APPROACH_SPEED, 0.8 * (p.x - 0.32)))
        command.linear.y = max(-APPROACH_SPEED, min(APPROACH_SPEED, 0.8 * p.y))
        self.approach_cmd = command
        self.approach_until = now + 1.5
        self.cmd_vel_pub.publish(command)
        return True

    def tick(self):
        now = self.now_seconds()
        # The RGB-D camera is needed only while idle, so the detector can find
        # the box for the next cycle.  Rendering throttles the physics to a
        # fraction of real time, so turn it off for the whole pick and for the
        # open-loop approach move.  The state is republished because a single
        # volatile sample can be lost.
        if now - self.camera_republish > 1.0:
            self.camera_pub.publish(Bool(data=(self.sequence is None
                                               and now >= self.approach_until)))
            self.camera_republish = now
        if self.sequence is None:
            if self.approach_next_box(now):
                self.status_pub.publish(String(data="approaching_box"))
                return
            if self.next_cycle_time is not None and now >= self.next_cycle_time:
                # Let the released box settle, then re-detect and start the next
                # cycle.  start_cycle fails until a fresh RGB-D pose arrives.
                if self.start_cycle(now):
                    self.get_logger().info(
                        f"starting cycle {self.cycle_index + 1}; waypoints="
                        f"{'side->main' if self.cycle_index % 2 else 'main->side'}")
            self.status_pub.publish(String(data=f"failed:{self.failed}" if self.failed else "idle"))
            return
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
            self.sequence = None
            self.cycle_index += 1
            cycles = int(self.get_parameter("cycles").value)
            if cycles == 0 or self.cycle_index < cycles:
                self.next_cycle_time = now + 3.0
                self.get_logger().info(
                    f"pick/place cycle {self.cycle_index} completed; waiting to repeat")
            else:
                self.get_logger().info(
                    f"pick/place loops finished after {self.cycle_index} cycles")


def main():
    rclpy.init()
    node = DualPickController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
