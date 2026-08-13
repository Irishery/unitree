"""RGB-D detector for the red tabletop box used by the MuJoCo task."""
import numpy as np

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker


class BoxDetector(Node):
    """Estimate the box centre from a red RGB mask and registered depth."""

    def __init__(self):
        super().__init__("g1_box_detector")
        self.rgb = self.depth = self.info = None
        self.pose_pub = self.create_publisher(PoseStamped, "/g1/perception/box_pose", 10)
        self.detected_pub = self.create_publisher(Bool, "/g1/perception/box_detected", 10)
        self.marker_pub = self.create_publisher(Marker, "/g1/perception/box_marker", 10)
        self.create_subscription(Image, "/camera/camera/color/image_raw", self.on_rgb, 10)
        self.create_subscription(Image, "/camera/camera/depth/image_rect_raw", self.on_depth, 10)
        self.create_subscription(CameraInfo, "/camera/camera/color/camera_info", self.on_info, 10)
        self.create_timer(1.0 / 15.0, self.detect)

    def on_rgb(self, message):
        if message.encoding.lower() == "rgb8" and message.step >= message.width * 3:
            self.rgb = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.width, 3).copy()

    def on_depth(self, message):
        if message.encoding == "32FC1" and message.step >= message.width * 4:
            self.depth = np.frombuffer(message.data, dtype=np.float32).reshape(message.height, message.width).copy()

    def on_info(self, message):
        self.info = message

    def publish_missing(self):
        self.detected_pub.publish(Bool(data=False))
        marker = Marker()
        marker.header.frame_id, marker.ns, marker.id = "d435_color_optical_frame", "detected_box", 0
        marker.action = Marker.DELETE
        self.marker_pub.publish(marker)

    def detect(self):
        if self.rgb is None or self.depth is None or self.info is None:
            return
        if self.rgb.shape[:2] != self.depth.shape:
            self.publish_missing()
            return
        red, green, blue = (self.rgb[:, :, index] for index in range(3))
        mask = (red > 170) & (green < 110) & (blue < 110) & ((red.astype(np.int16) - green) > 90)
        mask &= np.isfinite(self.depth) & (self.depth > 0.10) & (self.depth < 4.0)
        pixels = np.argwhere(mask)
        if len(pixels) < 80:
            self.publish_missing()
            return
        v, u = np.median(pixels, axis=0)
        u, v = int(round(u)), int(round(v))
        # MuJoCo returns ray distance; ROS pinhole coordinates use optical Z.
        ray_distance = float(np.median(self.depth[mask]))
        fx, fy, cx, cy = self.info.k[0], self.info.k[4], self.info.k[2], self.info.k[5]
        xn, yn = (u - cx) / fx, (v - cy) / fy
        z = ray_distance / np.sqrt(1.0 + xn * xn + yn * yn)
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "d435_color_optical_frame"
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = float(xn * z), float(yn * z), float(z)
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)
        self.detected_pub.publish(Bool(data=True))
        marker = Marker()
        marker.header, marker.ns, marker.id = pose.header, "detected_box", 0
        marker.type, marker.action = Marker.SPHERE, Marker.ADD
        marker.pose = pose.pose
        marker.scale.x = marker.scale.y = marker.scale.z = 0.07
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.1, 1.0, 0.1, 1.0
        self.marker_pub.publish(marker)


def main():
    rclpy.init()
    node = BoxDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
