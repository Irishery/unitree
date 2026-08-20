"""Periodic robot description publisher for RViz.

The stock ``/robot_description`` topic from robot_state_publisher is a
TRANSIENT_LOCAL latched message, and in this stack (Jazzy + Fast DDS in
Docker) the latched sample is never delivered to subscribers that appear
after the publisher (verified: neither RViz nor a raw transient-local
subscription receives it).  RViz therefore never loads the mesh model and
falls back to primitive shapes only.

RViz's RobotModel subscribes with TRANSIENT_LOCAL durability, so the
relay must also publish TRANSIENT_LOCAL (a volatile publisher is QoS
incompatible and receives nothing).  The trick is to republish the URDF
periodically: live samples are delivered to matched subscribers even
though the initial latched sample is not.

A second topic, ``/robot_description_lite``, carries a skeleton version
of the same URDF: every link keeps its frame but its mesh visual is
replaced by a small sphere.  Software-GL laptops render the 51 textured
DAE meshes too slowly; the skeleton model (a few dozen primitives) is
essentially free and still shows the full pose.  The ``lite`` RViz
profile subscribes to it.
"""

import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String


def build_skeleton_urdf(urdf: str, radius: float = 0.045) -> str:
    tree = ET.ElementTree(ET.fromstring(urdf))
    root = tree.getroot()
    for link in root.iter("link"):
        for tag in ("visual", "collision"):
            for child in link.findall(tag):
                link.remove(child)
        visual = ET.SubElement(link, "visual")
        ET.SubElement(visual, "geometry")
        sphere = ET.SubElement(visual.find("geometry"), "sphere")
        sphere.set("radius", str(radius))
    return ET.tostring(root, encoding="unicode")


class DescriptionRelay(Node):

    def __init__(self):
        super().__init__("description_relay")
        self.declare_parameter("robot_description", "")
        self.declare_parameter("publish_rate", 1.0)
        urdf = self.get_parameter("robot_description").value
        if not urdf:
            self.get_logger().error("robot_description parameter is empty")
            raise SystemExit(1)
        rate = float(self.get_parameter("publish_rate").value)
        qos = QoSProfile(
            depth=5,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self._pub = self.create_publisher(String, "/robot_description_viz", qos)
        self._pub_lite = self.create_publisher(String, "/robot_description_lite", qos)
        self._msg = String(data=urdf)
        self._msg_lite = String(data=build_skeleton_urdf(urdf))
        self.create_timer(1.0 / max(rate, 0.1), self._tick)
        self.get_logger().info(
            f"Publishing URDF ({len(urdf)} chars) on /robot_description_viz "
            f"and skeleton ({len(self._msg_lite.data)} chars) on "
            f"/robot_description_lite at {rate} Hz")

    def _tick(self):
        self._pub.publish(self._msg)
        self._pub_lite.publish(self._msg_lite)


def main():
    rclpy.init()
    node = DescriptionRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            # The SIGINT handler may have shut the context down already.
            pass
