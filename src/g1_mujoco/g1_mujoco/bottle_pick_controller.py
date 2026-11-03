"""ROS executor for the stationary single-arm bottle pick sequence."""
import math
import os
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String

from .bottle_sequence import SmoothSequence, build_pick_plan
from .grasp_planning import (
    ARM_JOINTS, BASE_POSE, FINGER_OPEN, GraspKinematics, GraspParams, HAND_JOINTS)


class BottlePickController(Node):
    """Plan and execute one stationary one-hand bottle pick.

    The selected arm performs the pick while the other arm stays in READY.
    The base is never commanded: the robot stands at the table for the whole
    sequence.  The RGB-D pose is required before the first motion.
    """

    def __init__(self):
        super().__init__("g1_bottle_pick_controller")
        self.declare_parameter("pose_timeout", 0.75)
        self.declare_parameter("model_z_offset", float(BASE_POSE[2]))
        self.declare_parameter("bottle_diameter", 0.062)
        self.declare_parameter("bottle_height", 0.235)
        self.declare_parameter("grasp_side", "right")
        self.grasp_side = str(self.get_parameter("grasp_side").value)
        if self.grasp_side not in ("left", "right"):
            raise ValueError("grasp_side must be 'left' or 'right'")
        description = Path(os.environ.get(
            "G1_DESCRIPTION_DIR", "/opt/unitree_ros/robots/g1_description"))
        self.kinematics = GraspKinematics(description / "g1_29dof_with_dex3_bottle.xml")
        self.grasp_params = GraspParams(
            bottle_diameter=float(self.get_parameter("bottle_diameter").value),
            bottle_height=float(self.get_parameter("bottle_height").value))
        self.pose = None
        self.pose_received = None
        self.size = None
        self.size_received = None
        self.joints = {}
        self.sequence = None
        self.failed = None
        self.camera_republish = 0.0
        self.arm_pubs = {
            name: self.create_publisher(
                Float64, f"/g1/mujoco/joints/{name}/command", 10)
            for names in ARM_JOINTS.values() for name in names
        }
        self.hand_pubs = {
            side: self.create_publisher(JointState, f"/g1/dex3/{side}/command", 10)
            for side in ARM_JOINTS
        }
        self.camera_pub = self.create_publisher(Bool, "/g1/mujoco/camera_enable", 10)
        self.status_pub = self.create_publisher(String, "/g1/pick/status", 10)
        self.create_subscription(Bool, "/g1/pick/start", self.on_start, 10)
        self.create_subscription(Bool, "/g1/pick/cancel", self.on_cancel, 10)
        self.create_subscription(PoseStamped, "/g1/perception/bottle_pose", self.on_pose, 10)
        self.create_subscription(
            Vector3Stamped, "/g1/perception/bottle_size", self.on_size, 10)
        self.create_subscription(JointState, "/g1/joint_states", self.on_joints, 10)
        self.create_timer(0.02, self.tick)
        self.get_logger().info(
            f"Bottle pick controller ready: {self.grasp_side} hand, stationary "
            "base, RGB-D pose required; send std_msgs/Bool true on /g1/pick/start")

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_pose(self, message):
        self.pose, self.pose_received = message, self.now_seconds()

    def on_size(self, message):
        self.size, self.size_received = message, self.now_seconds()

    def on_joints(self, message):
        self.joints.update(zip(message.name, message.position))

    def on_cancel(self, message):
        if message.data:
            self.sequence, self.failed = None, "cancelled"
            self.camera_pub.publish(Bool(data=True))
            self.status_pub.publish(String(data="cancelled"))

    def on_start(self, message):
        if not message.data or self.sequence is not None:
            return
        self.start_cycle(self.now_seconds())

    def start_cycle(self, now):
        timeout = float(self.get_parameter("pose_timeout").value)
        if self.pose is None or self.pose_received is None or now - self.pose_received > timeout:
            self.failed = "no_fresh_rgbd_pose"
            self.get_logger().error("refusing bottle pick: no fresh RGB-D bottle pose")
            return False
        if (self.size is None or self.size_received is None
                or now - self.size_received > timeout):
            self.failed = "no_fresh_bottle_size"
            self.get_logger().error("refusing bottle pick: no fresh bottle size estimate")
            return False
        diameter = float(self.size.vector.x)
        height = float(self.size.vector.y)
        confidence = float(self.size.vector.z)
        if confidence < 0.20:
            self.failed = f"low_bottle_size_confidence:{confidence:.2f}"
            self.get_logger().error(self.failed)
            return False
        if not (0.030 <= diameter <= 0.090 and 0.080 <= height <= 0.350):
            self.failed = f"bottle_size_out_of_range:{diameter:.3f}x{height:.3f}"
            self.get_logger().error(self.failed)
            return False
        if any(name not in self.joints for side in ARM_JOINTS for name in ARM_JOINTS[side]):
            self.failed = "no_joint_state"
            self.get_logger().error("refusing bottle pick: arm joint state is incomplete")
            return False
        p, q = self.pose.pose.position, self.pose.pose.orientation
        centre = np.array([p.x, p.y, p.z], dtype=float)
        if self.pose.header.frame_id == "pelvis":
            centre[2] += float(self.get_parameter("model_z_offset").value)
        yaw = math.atan2(2.0 * (q.w*q.z + q.x*q.y),
                         1.0 - 2.0 * (q.y*q.y + q.z*q.z))
        # A larger bottle needs less finger closure; keep the same absolute
        # lateral clearance but honour the measured body diameter.
        close_scale = max(0.65, min(1.0, 0.95 * 0.062 / diameter))
        grasp_params = GraspParams(
            bottle_diameter=diameter, bottle_height=height,
            finger_close_scale=close_scale)
        try:
            plan = build_pick_plan(
                self.kinematics, centre, yaw, params=grasp_params,
                side=self.grasp_side)
        except RuntimeError as error:
            self.failed = f"planning_failed:{error}"
            self.get_logger().error(self.failed)
            return False
        initial_arms = {
            s: np.array([self.joints[n] for n in ARM_JOINTS[s]]) for s in ARM_JOINTS}
        initial_hands = {
            s: np.array([self.joints.get(n, FINGER_OPEN[s][i])
                         for i, n in enumerate(HAND_JOINTS[s])])
            for s in ARM_JOINTS}
        self.sequence = SmoothSequence(plan, initial_arms, initial_hands)
        self.sequence.started = now
        self.failed = None
        self.camera_pub.publish(Bool(data=False))
        self.get_logger().info(
            f"bottle pick planned from RGB-D: centre={np.round(centre, 3).tolist()}, "
            f"size={diameter:.3f}x{height:.3f} m (conf {confidence:.2f}), "
            f"close_scale={close_scale:.2f}, hand={self.grasp_side}; "
            "robot remains stationary")
        return True

    def publish_commands(self, arms, hands):
        for side in ARM_JOINTS:
            for name, value in zip(ARM_JOINTS[side], arms[side]):
                self.arm_pubs[name].publish(Float64(data=float(value)))
            message = JointState()
            message.name = HAND_JOINTS[side]
            message.position = [float(v) for v in hands[side]]
            self.hand_pubs[side].publish(message)

    def tick(self):
        now = self.now_seconds()
        if self.sequence is None:
            status = f"failed:{self.failed}" if self.failed else "idle"
            self.status_pub.publish(String(data=status))
            return
        # Rendering throttles physics; keep the camera off during the motion
        # and republish the request because a single volatile sample can be lost.
        if now - self.camera_republish > 1.0:
            self.camera_pub.publish(Bool(data=False))
            self.camera_republish = now
        arms, hands = self.sequence.sample(now)
        self.publish_commands(arms, hands)
        stage = self.sequence.stage
        self.status_pub.publish(String(data=stage))
        if self.sequence.done:
            self.camera_pub.publish(Bool(data=True))
            self.sequence = None
            self.failed = None
            self.status_pub.publish(String(data="done"))
            self.get_logger().info(
                f"bottle pick/place sequence finished with {self.grasp_side} hand; "
                "base never moved")


def main():
    rclpy.init()
    node = BottlePickController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
