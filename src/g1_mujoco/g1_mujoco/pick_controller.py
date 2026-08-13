"""Conservative reference pick state machine for the MuJoCo DEX3 task.

It is deliberately separate from learning: its trajectory and resulting
contact data form a reproducible baseline and demonstration source for RL.
"""
from enum import Enum

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, Int32, String


ARM_JOINTS = {
    "left": ["left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
             "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint"],
    "right": ["right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
              "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint"],
}


class Stage(Enum):
    IDLE = "idle"
    PREGRASP = "pregrasp"
    CLOSE = "close"
    LIFT = "lift"
    DONE = "done"
    FAILED = "failed"


class PickController(Node):
    """Publish a guarded bimanual baseline trajectory after an explicit start."""

    def __init__(self):
        super().__init__("g1_pick_controller")
        self.stage = Stage.IDLE
        self.stage_started = self.get_clock().now()
        self.box_pose = None
        self.contacts = 0
        self.grasped = False
        self.command_pubs = {
            name: self.create_publisher(Float64, f"/g1/mujoco/joints/{name}/command", 10)
            for joints in ARM_JOINTS.values() for name in joints
        }
        self.hand_pubs = {
            side: self.create_publisher(JointState, f"/g1/dex3/{side}/command", 10)
            for side in ARM_JOINTS
        }
        self.status_pub = self.create_publisher(String, "/g1/pick/status", 10)
        self.create_subscription(Bool, "/g1/pick/start", self.on_start, 10)
        self.create_subscription(PoseStamped, "/g1/perception/box_pose", self.on_box, 10)
        self.create_subscription(Int32, "/g1/mujoco/hand_box_contacts", self.on_contacts, 10)
        self.create_subscription(Bool, "/g1/mujoco/physical_grasp", self.on_grasp, 10)
        self.create_timer(0.05, self.tick)

    def on_box(self, message):
        self.box_pose = message

    def on_contacts(self, message):
        self.contacts = message.data

    def on_grasp(self, message):
        self.grasped = message.data

    def on_start(self, message):
        if not message.data or self.stage not in (Stage.IDLE, Stage.DONE, Stage.FAILED):
            return
        if self.box_pose is None:
            self.stage = Stage.FAILED
            self.get_logger().error("refusing pick: no RGB-D box pose")
        else:
            self.stage = Stage.PREGRASP
        self.stage_started = self.get_clock().now()

    def elapsed(self):
        return (self.get_clock().now() - self.stage_started).nanoseconds / 1e9

    def set_stage(self, stage):
        self.stage, self.stage_started = stage, self.get_clock().now()
        self.get_logger().info(f"pick stage: {stage.value}")

    def command_arm(self, side, values):
        for name, value in zip(ARM_JOINTS[side], values):
            self.command_pubs[name].publish(Float64(data=float(value)))

    def command_hand(self, side, positions):
        message = JointState()
        message.position = positions
        self.hand_pubs[side].publish(message)

    def tick(self):
        self.status_pub.publish(String(data=self.stage.value))
        if self.stage == Stage.PREGRASP:
            # Validated in the current MJCF: fingertips are outside the box
            # sides (y about +/-0.30 m) and above its top.  The old narrow
            # roll target put the middle fingers into the tabletop.
            self.command_arm("left", [-0.65, 0.55, -0.20, 0.87, 0.0, 0.02, 0.0])
            self.command_arm("right", [-0.65, -0.55, 0.20, 0.87, 0.0, 0.02, 0.0])
            self.command_hand("left", [0.0, 0.15, 0.15, -0.15, -0.15, -0.15, -0.15])
            self.command_hand("right", [0.0, -0.15, -0.15, 0.15, 0.15, 0.15, 0.15])
            if self.elapsed() > 3.0:
                self.set_stage(Stage.CLOSE)
        elif self.stage == Stage.CLOSE:
            self.command_hand("left", [0.0, 0.55, 0.80, -0.90, -1.05, -0.90, -1.05])
            self.command_hand("right", [0.0, -0.55, -0.80, 0.90, 1.05, 0.90, 1.05])
            if self.grasped:
                self.set_stage(Stage.LIFT)
            elif self.elapsed() > 3.0:
                self.set_stage(Stage.FAILED)
        elif self.stage == Stage.LIFT:
            self.command_arm("left", [-0.90, 0.18, -0.20, 0.95, 0.0, 0.0, 0.0])
            self.command_arm("right", [-0.90, -0.18, 0.20, 0.95, 0.0, 0.0, 0.0])
            if self.elapsed() > 2.0:
                self.set_stage(Stage.DONE)


def main():
    rclpy.init()
    node = PickController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
