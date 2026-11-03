"""RGB-D pose detector for the known red tabletop bottle (no fiducials)."""
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker

from .bottle_geometry import (
    BOTTLE_DIAMETER, BOTTLE_HEIGHT, backproject, fit_bottle_pose,
    red_bottle_mask, transform_points)


def quaternion_matrix(x, y, z, w):
    q = np.asarray([w, x, y, z], dtype=np.float64)
    q /= max(np.linalg.norm(q), 1e-12)
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])


class BottleDetector(Node):
    """Estimate the upright bottle centre from segmented RGB-D geometry."""

    def __init__(self):
        super().__init__("g1_bottle_detector")
        self.declare_parameter("output_frame", "pelvis")
        self.declare_parameter("max_frame_age", 0.20)
        self.declare_parameter("bottle_diameter", BOTTLE_DIAMETER)
        self.declare_parameter("bottle_height", BOTTLE_HEIGHT)
        self.rgb = self.depth = self.info = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose_pub = self.create_publisher(PoseStamped, "/g1/perception/bottle_pose", 10)
        self.detected_pub = self.create_publisher(Bool, "/g1/perception/bottle_detected", 10)
        self.size_pub = self.create_publisher(
            Vector3Stamped, "/g1/perception/bottle_size", 10)
        self.marker_pub = self.create_publisher(Marker, "/g1/perception/bottle_marker", 10)
        self.create_subscription(Image, "/camera/camera/color/image_raw", self.on_rgb, 10)
        self.create_subscription(Image, "/camera/camera/depth/image_rect_raw", self.on_depth, 10)
        self.create_subscription(CameraInfo, "/camera/camera/color/camera_info", self.on_info, 10)
        self.create_timer(1.0 / 15.0, self.detect)

    def on_rgb(self, message):
        if message.encoding.lower() == "rgb8" and message.step >= message.width * 3:
            self.rgb = (message, np.frombuffer(message.data, dtype=np.uint8)
                        .reshape(message.height, message.width, 3).copy())

    def on_depth(self, message):
        if message.encoding == "32FC1" and message.step >= message.width * 4:
            self.depth = (message, np.frombuffer(message.data, dtype=np.float32)
                          .reshape(message.height, message.width).copy())

    def on_info(self, message):
        self.info = message

    def publish_missing(self):
        self.detected_pub.publish(Bool(data=False))
        marker = Marker()
        marker.header.frame_id = str(self.get_parameter("output_frame").value)
        marker.ns, marker.id, marker.action = "detected_bottle", 0, Marker.DELETE
        self.marker_pub.publish(marker)

    def detect(self):
        if self.rgb is None or self.depth is None or self.info is None:
            return
        rgb_msg, rgb = self.rgb
        depth_msg, depth = self.depth
        if rgb.shape[:2] != depth.shape:
            self.publish_missing()
            return
        delta = abs((Time.from_msg(rgb_msg.header.stamp)
                     - Time.from_msg(depth_msg.header.stamp)).nanoseconds) * 1e-9
        if delta > float(self.get_parameter("max_frame_age").value):
            self.publish_missing()
            return
        points, _ = backproject(red_bottle_mask(rgb), depth, self.info.k[0], self.info.k[4],
                                self.info.k[2], self.info.k[5])
        if len(points) == 0:
            self.publish_missing()
            return
        output_frame = str(self.get_parameter("output_frame").value)
        source_frame = depth_msg.header.frame_id or "d435_color_optical_frame"
        try:
            transform = self.tf_buffer.lookup_transform(output_frame, source_frame, Time())
        except TransformException as error:
            self.get_logger().warn(
                f"bottle transform unavailable: {error}", throttle_duration_sec=2.0)
            self.publish_missing()
            return
        q, t = transform.transform.rotation, transform.transform.translation
        points = transform_points(points, quaternion_matrix(q.x, q.y, q.z, q.w),
                                  np.array([t.x, t.y, t.z]))
        estimate = fit_bottle_pose(
            points,
            diameter=float(self.get_parameter("bottle_diameter").value),
            height=float(self.get_parameter("bottle_height").value))
        if estimate is None:
            self.publish_missing()
            return
        pose = PoseStamped()
        pose.header.stamp, pose.header.frame_id = rgb_msg.header.stamp, output_frame
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(
            float, estimate["centre"])
        pose.pose.orientation.w = 1.0  # upright, rotationally symmetric bottle
        self.pose_pub.publish(pose)
        self.detected_pub.publish(Bool(data=True))
        size = Vector3Stamped()
        size.header = pose.header
        # x=diameter, y=height, z=confidence (0..1)
        size.vector.x = float(estimate["diameter"])
        size.vector.y = float(estimate["height"])
        size.vector.z = float(estimate.get("confidence", 0.0))
        self.size_pub.publish(size)
        marker = Marker()
        marker.header, marker.ns, marker.id = pose.header, "detected_bottle", 0
        marker.type, marker.action, marker.pose = Marker.CYLINDER, Marker.ADD, pose.pose
        marker.scale.x = marker.scale.y = float(estimate["diameter"])
        marker.scale.z = float(estimate["height"])
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.1, 1.0, 0.1, 0.45
        self.marker_pub.publish(marker)


def main():
    rclpy.init()
    node = BottleDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
