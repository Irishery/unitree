"""ROS 2 adapter for a contact-only MuJoCo G1 DEX3-1 tabletop scene."""
from pathlib import Path
import os

import mujoco
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, JointState, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Float64, Header, Int32


HAND_JOINTS = {
    "left": [
        "left_hand_thumb_0_joint", "left_hand_thumb_1_joint", "left_hand_thumb_2_joint",
        "left_hand_middle_0_joint", "left_hand_middle_1_joint",
        "left_hand_index_0_joint", "left_hand_index_1_joint",
    ],
    "right": [
        "right_hand_thumb_0_joint", "right_hand_thumb_1_joint", "right_hand_thumb_2_joint",
        "right_hand_middle_0_joint", "right_hand_middle_1_joint",
        "right_hand_index_0_joint", "right_hand_index_1_joint",
    ],
}

# Visual "arms at sides" rest pose.  The official MJCF's elbow link has a
# mechanical offset, so elbow=0 projects the forearm forward.  The previous
# vertical arm pose is retained; only a small symmetric shoulder roll moves
# the arms outside the torso and thighs in the visual model.
ARMS_AT_SIDES = {
    "left_shoulder_pitch_joint": 0.0,
    "left_shoulder_roll_joint": 0.25,
    "left_shoulder_yaw_joint": 0.0,
    "left_elbow_joint": 1.50,
    "left_wrist_roll_joint": 0.0,
    "left_wrist_pitch_joint": 0.0,
    "left_wrist_yaw_joint": 0.0,
    "left_hand_thumb_0_joint": 0.0,
    "left_hand_thumb_1_joint": 0.25,
    "left_hand_thumb_2_joint": 0.45,
    "left_hand_middle_0_joint": -0.55,
    "left_hand_middle_1_joint": -0.70,
    "left_hand_index_0_joint": -0.55,
    "left_hand_index_1_joint": -0.70,
    "right_shoulder_pitch_joint": 0.0,
    "right_shoulder_roll_joint": -0.25,
    "right_shoulder_yaw_joint": 0.0,
    "right_elbow_joint": 1.50,
    "right_wrist_roll_joint": 0.0,
    "right_wrist_pitch_joint": 0.0,
    "right_wrist_yaw_joint": 0.0,
    "right_hand_thumb_0_joint": 0.0,
    "right_hand_thumb_1_joint": -0.25,
    "right_hand_thumb_2_joint": -0.45,
    "right_hand_middle_0_joint": 0.55,
    "right_hand_middle_1_joint": 0.70,
    "right_hand_index_0_joint": 0.55,
    "right_hand_index_1_joint": 0.70,
}


class G1Mujoco(Node):
    def __init__(self):
        super().__init__("g1_mujoco")
        self.declare_parameter("viewer", True)
        description = Path(os.environ.get("G1_DESCRIPTION_DIR", "/opt/unitree_ros/robots/g1_description"))
        self.model = mujoco.MjModel.from_xml_path(str(description / "g1_29dof_with_dex3_tabletop.xml"))
        self.data = mujoco.MjData(self.model)
        self.names = [
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            for i in range(self.model.njnt)
            if self.model.jnt_type[i] in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE)
        ]
        self.names = [name for name in self.names if name]
        self.joint_ids = {name: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
                          for name in self.names}
        self.actuator_ids = {name: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
                             for name in self.names}
        for name, position in ARMS_AT_SIDES.items():
            self.data.qpos[self.model.jnt_qposadr[self.joint_ids[name]]] = position
        mujoco.mj_forward(self.model, self.data)
        self.target = {name: self.qpos(name) for name in self.names}
        self.step_count = 0
        self.declare_parameter("publish_camera", True)
        self.publisher = self.create_publisher(JointState, "/g1/joint_states", 10)
        self.contacts_pub = self.create_publisher(Int32, "/g1/mujoco/hand_box_contacts", 10)
        self.grasp_pub = self.create_publisher(Bool, "/g1/mujoco/physical_grasp", 10)
        self.color_pub = self.create_publisher(Image, "/camera/camera/color/image_raw", 10)
        self.depth_pub = self.create_publisher(Image, "/camera/camera/depth/image_rect_raw", 10)
        self.color_info_pub = self.create_publisher(CameraInfo, "/camera/camera/color/camera_info", 10)
        self.depth_info_pub = self.create_publisher(CameraInfo, "/camera/camera/depth/camera_info", 10)
        self.points_pub = self.create_publisher(PointCloud2, "/camera/camera/depth/color/points", 10)
        for name in self.names:
            self.create_subscription(Float64, f"/g1/mujoco/joints/{name}/command",
                                     lambda msg, joint=name: self.set_target(joint, msg.data), 10)
        for side, joints in HAND_JOINTS.items():
            self.create_subscription(JointState, f"/g1/dex3/{side}/command",
                                     lambda msg, target=joints: self.set_hand(target, msg), 10)
        self.viewer = None
        if self.get_parameter("viewer").value:
            from mujoco import viewer as mujoco_viewer
            self.viewer = mujoco_viewer.launch_passive(self.model, self.data)
        self.renderer = None
        if self.get_parameter("publish_camera").value:
            self.renderer = mujoco.Renderer(self.model, width=640, height=480)
        self.timer = self.create_timer(0.002, self.step)
        self.get_logger().info("MuJoCo DEX3 contact simulation: no attach or weld is used for pickup_box")

    def qpos(self, name):
        return float(self.data.qpos[self.model.jnt_qposadr[self.joint_ids[name]]])

    def qvel(self, name):
        return float(self.data.qvel[self.model.jnt_dofadr[self.joint_ids[name]]])

    def set_target(self, name, value):
        if name in self.target and abs(value) < 10.0:
            rng = self.model.jnt_range[self.joint_ids[name]]
            self.target[name] = max(float(rng[0]), min(float(rng[1]), value))

    def set_hand(self, names, message):
        if len(message.position) != 7:
            self.get_logger().error("DEX3 command must contain 7 ordered positions")
            return
        for name, value in zip(names, message.position):
            self.set_target(name, value)

    def step(self):
        for name, actuator in self.actuator_ids.items():
            finger = "_hand_" in name
            kp, kd = (3.0, 0.18) if finger else (35.0, 1.8)
            limit = 0.9 if finger else 18.0
            torque = kp * (self.target[name] - self.qpos(name)) - kd * self.qvel(name)
            self.data.ctrl[actuator] = max(-limit, min(limit, torque))
        mujoco.mj_step(self.model, self.data)
        self.step_count += 1
        if self.step_count % 10 == 0:
            self.publish_state()
        if self.renderer is not None and self.step_count % 17 == 0:
            self.publish_camera()
        if self.viewer is not None and self.step_count % 10 == 0:
            self.viewer.sync()

    def publish_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.names
        msg.position = [self.qpos(name) for name in self.names]
        msg.velocity = [self.qvel(name) for name in self.names]
        self.publisher.publish(msg)
        contacts = 0
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            bodies = [mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY,
                                        self.model.geom_bodyid[geom]) or ""
                      for geom in (contact.geom1, contact.geom2)]
            if "pickup_box" in bodies and any("_hand_" in body for body in bodies):
                contacts += 1
        self.contacts_pub.publish(Int32(data=contacts))
        # At least two distinct contact points is a measurable contact grasp
        # signal. It does not alter physics or attach the object.
        self.grasp_pub.publish(Bool(data=contacts >= 2))

    def camera_info(self, stamp):
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = "d435_color_optical_frame"
        info.width, info.height = 640, 480
        fovy = np.deg2rad(69.0)
        fy = info.height / (2.0 * np.tan(fovy * 0.5))
        fx = fy
        cx, cy = info.width * 0.5, info.height * 0.5
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info, fx, fy, cx, cy

    def publish_camera(self):
        self.renderer.update_scene(self.data, camera="d435i")
        rgb = self.renderer.render()
        self.renderer.enable_depth_rendering()
        depth = self.renderer.render().astype(np.float32)
        self.renderer.disable_depth_rendering()
        stamp = self.get_clock().now().to_msg()
        info, fx, fy, cx, cy = self.camera_info(stamp)
        self.color_info_pub.publish(info)
        self.depth_info_pub.publish(info)
        color = Image()
        color.header = info.header
        color.height, color.width = rgb.shape[:2]
        color.encoding, color.is_bigendian = "rgb8", False
        color.step = color.width * 3
        color.data = rgb.tobytes()
        self.color_pub.publish(color)
        depth_msg = Image()
        depth_msg.header = info.header
        depth_msg.height, depth_msg.width = depth.shape
        depth_msg.encoding, depth_msg.is_bigendian = "32FC1", False
        depth_msg.step = depth_msg.width * 4
        depth_msg.data = depth.tobytes()
        self.depth_pub.publish(depth_msg)
        # Decimate to keep the ROS point cloud lightweight while preserving
        # the optical frame convention used by RealSense-compatible tools.
        v, u = np.mgrid[0:depth.shape[0]:4, 0:depth.shape[1]:4]
        distance = depth[::4, ::4]
        valid = np.isfinite(distance) & (distance > 0.05) & (distance < 5.0)
        # MuJoCo returns a ray distance, whereas ROS pinhole projection uses
        # optical Z.  Convert before producing the PointCloud2 message.
        xn = (u[valid] - cx) / fx
        yn = (v[valid] - cy) / fy
        z = distance[valid] / np.sqrt(1.0 + xn * xn + yn * yn)
        x = xn * z
        y = yn * z
        points = np.column_stack((x, y, z)).astype(np.float32)
        # A zero stamp asks TF consumers such as RViz for the latest available
        # torso transform.  Rendering takes longer than a physics step, so a
        # wall-clock stamp here can otherwise be a few milliseconds newer than
        # the latest /joint_states-derived transform and flicker to Error.
        cloud_header = Header()
        cloud_header.frame_id = info.header.frame_id
        self.points_pub.publish(point_cloud2.create_cloud_xyz32(cloud_header, points))

    def destroy_node(self):
        if self.viewer is not None:
            self.viewer.close()
        if self.renderer is not None:
            self.renderer.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = G1Mujoco()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
