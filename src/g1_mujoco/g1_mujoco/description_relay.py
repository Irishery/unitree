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
replaced by a small capsule/oval.  Software-GL laptops render the 51
textured DAE meshes too slowly; the oval model (a few dozen primitives)
is essentially free and still shows the full pose.  The ``lite`` RViz
profile subscribes to it.
"""

import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String


LITE_RGBA_DARK = "0.20 0.20 0.20 1"
LITE_RGBA_LIGHT = "0.70 0.70 0.70 1"
LITE_RGBA_SENSOR = "0.05 0.05 0.05 1"


def lite_shape(name: str):
    """Return an approximate link capsule: axis, length, radius, xyz, rgba.

    URDF has no ellipsoid/capsule primitive, so RViz gets a capsule built
    from one cylinder plus two spheres.  The dimensions are intentionally
    conservative: this is only a lightweight visual model, not collision
    geometry.
    """
    rgba = LITE_RGBA_DARK if any(part in name for part in ("pelvis", "head", "ankle_roll")) else LITE_RGBA_LIGHT
    if name == "torso_link":
        return "z", 0.36, 0.105, "0 0 0.16", LITE_RGBA_LIGHT
    if name in ("head_link", "logo_link"):
        return "z", 0.08, 0.075, "0 0 0.03", LITE_RGBA_SENSOR
    if name.startswith("pelvis"):
        return "y", 0.16, 0.080, "0 0 -0.04", LITE_RGBA_DARK
    if name.startswith("waist"):
        return "z", 0.06, 0.055, "0 0 0.02", LITE_RGBA_LIGHT
    if "hip_pitch" in name or "hip_roll" in name:
        return "y", 0.08, 0.050, "0 0 0", rgba
    if "hip_yaw" in name:
        return "z", 0.18, 0.050, "0 0 -0.07", rgba
    if "knee" in name:
        return "z", 0.24, 0.047, "0 0 -0.12", rgba
    if "ankle_pitch" in name:
        return "z", 0.07, 0.034, "0 0 -0.02", rgba
    if "ankle_roll" in name:
        return "x", 0.20, 0.035, "0.03 0 -0.025", LITE_RGBA_DARK
    if "shoulder_pitch" in name or "shoulder_roll" in name:
        return "y", 0.08, 0.042, "0 0 0", rgba
    if "shoulder_yaw" in name:
        return "z", 0.15, 0.040, "0 0 -0.06", rgba
    if "elbow" in name:
        return "x", 0.15, 0.038, "0.055 0 0", rgba
    if "wrist" in name:
        return "x", 0.07, 0.028, "0.03 0 0", rgba
    if "hand_palm" in name:
        return "x", 0.10, 0.033, "0.05 0 0", rgba
    if "_hand_" in name:
        return "x", 0.055, 0.018, "0.025 0 0", rgba
    if name.startswith("d435") or name.startswith("mid360") or name.startswith("imu"):
        return "z", 0.02, 0.025, "0 0 0", LITE_RGBA_SENSOR
    return "z", 0.06, 0.030, "0 0 0", rgba


def axis_pose(axis: str, offset: float) -> str:
    if axis == "x":
        return f"{offset} 0 0"
    if axis == "y":
        return f"0 {offset} 0"
    return f"0 0 {offset}"


def cylinder_rpy(axis: str) -> str:
    # URDF cylinders are aligned along local Z.
    if axis == "x":
        return "0 1.57079632679 0"
    if axis == "y":
        return "1.57079632679 0 0"
    return "0 0 0"


def add_material(visual, rgba: str):
    material = ET.SubElement(visual, "material")
    material.set("name", "lite_" + rgba.replace(" ", "_").replace(".", ""))
    color = ET.SubElement(material, "color")
    color.set("rgba", rgba)


def add_capsule_visual(link, axis: str, length: float, radius: float, xyz: str, rgba: str):
    body = ET.SubElement(link, "visual")
    ET.SubElement(body, "origin", {"xyz": xyz, "rpy": cylinder_rpy(axis)})
    body_geometry = ET.SubElement(body, "geometry")
    ET.SubElement(body_geometry, "cylinder", {"radius": f"{radius:.4f}", "length": f"{length:.4f}"})
    add_material(body, rgba)

    half = length * 0.5
    xyz_values = [float(value) for value in xyz.split()]
    for sign in (-1.0, 1.0):
        cap_xyz = xyz_values[:]
        if axis == "x":
            cap_xyz[0] += sign * half
        elif axis == "y":
            cap_xyz[1] += sign * half
        else:
            cap_xyz[2] += sign * half
        cap = ET.SubElement(link, "visual")
        ET.SubElement(cap, "origin", {"xyz": " ".join(f"{value:.5f}" for value in cap_xyz), "rpy": "0 0 0"})
        cap_geometry = ET.SubElement(cap, "geometry")
        ET.SubElement(cap_geometry, "sphere", {"radius": f"{radius:.4f}"})
        add_material(cap, rgba)


def build_skeleton_urdf(urdf: str) -> str:
    tree = ET.ElementTree(ET.fromstring(urdf))
    root = tree.getroot()
    for link in root.iter("link"):
        for tag in ("visual", "collision"):
            for child in link.findall(tag):
                link.remove(child)
        add_capsule_visual(link, *lite_shape(link.get("name", "")))
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
            f"and oval skeleton ({len(self._msg_lite.data)} chars) on "
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
