"""ROS executor adapter for the perception-driven bimanual pick sequence."""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String

from .bimanual_sequence import SmoothSequence, build_pick_plan
from .grasp_planning import ARM_JOINTS, BASE_POSE, FINGER_OPEN, GraspKinematics, HAND_JOINTS


class DualPickController(Node):
    """Own scenario state while delegating physics/hardware I/O to ROS topics."""

    def __init__(self):
        super().__init__("g1_dual_pick_controller")
        self.declare_parameter("pose_timeout", 0.75)
        self.declare_parameter("model_z_offset", float(BASE_POSE[2]))
        self.kinematics = GraspKinematics()
        self.pose = None
        self.pose_received = None
        self.joints = {}
        self.sequence = None
        self.failed = None
        self.arm_pubs = {name: self.create_publisher(
            Float64, f"/g1/mujoco/joints/{name}/command", 10)
            for names in ARM_JOINTS.values() for name in names}
        self.hand_pubs = {side: self.create_publisher(
            JointState, f"/g1/dex3/{side}/command", 10) for side in ARM_JOINTS}
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
        try:
            plan = build_pick_plan(self.kinematics, centre, yaw)
        except RuntimeError as error:
            self.failed = f"planning_failed:{error}"
            self.get_logger().error(self.failed)
            return
        initial_arms = {s: np.array([self.joints[n] for n in ARM_JOINTS[s]]) for s in ARM_JOINTS}
        initial_hands = {s: np.array([self.joints.get(n, FINGER_OPEN[s][i])
                                      for i, n in enumerate(HAND_JOINTS[s])]) for s in ARM_JOINTS}
        self.sequence = SmoothSequence(plan, initial_arms, initial_hands)
        self.sequence.started = now
        self.failed = None
        self.get_logger().info(f"planned bimanual pick from RGB-D: centre={centre}, yaw={yaw:.3f}")

    def publish_commands(self, arms, hands):
        for side in ARM_JOINTS:
            for name, value in zip(ARM_JOINTS[side], arms[side]):
                self.arm_pubs[name].publish(Float64(data=float(value)))
            message = JointState()
            message.name = HAND_JOINTS[side]
            message.position = [float(v) for v in hands[side]]
            self.hand_pubs[side].publish(message)

    def tick(self):
        if self.sequence is None:
            self.status_pub.publish(String(data=f"failed:{self.failed}" if self.failed else "idle"))
            return
        stage = self.sequence.stage
        arms, hands = self.sequence.sample(self.now_seconds())
        self.publish_commands(arms, hands)  # continuous even during settle/hold
        self.status_pub.publish(String(data=stage))
        if self.sequence.done:
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
