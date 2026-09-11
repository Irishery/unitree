#!/usr/bin/env python3
"""Add a contact-only tabletop scene to Unitree's official G1+DEx3 MJCF."""
import argparse
import math
import xml.etree.ElementTree as ET
from pathlib import Path

TABLETOP_TABLE_HALF_LENGTH = 0.30
TABLETOP_TABLE_HALF_WIDTH = 0.70
TABLETOP_TABLE_CENTER_X = 0.15 + TABLETOP_TABLE_HALF_LENGTH
# Second tabletop immediately to the robot's right (negative Y), flush with the
# main table's right edge and at the same height.  It is a separate static body
# so the manipulation scene has a second work surface without touching the
# validated centred grasp on the main table.
# Same slab dimensions as the main table (0.60 x 1.40 m), only rotated.
SIDE_TABLE_HALF_LENGTH = TABLETOP_TABLE_HALF_LENGTH
SIDE_TABLE_HALF_WIDTH = TABLETOP_TABLE_HALF_WIDTH
SIDE_TABLE_GAP = 0.005
SIDE_TABLE_YAW_DEG = 90.0
NAV_TABLE_HALF_LENGTH = 0.18
NAV_TABLE_HALF_WIDTH = 0.35
NAV_TABLE_CENTER_X = 0.95
TABLE_TOP_HEIGHT = 0.755
TABLE_THICKNESS_HALF = 0.04
BOX_LENGTH = 0.255
BOX_WIDTH = 0.370
BOX_HEIGHT = 0.090
RAILED_BOX_LENGTH = 0.150
RAILED_BOX_WIDTH = 0.250
RAILED_BOX_HEIGHT = 0.140
# Optional, visible side rails for a box that has the same physical feature on
# hardware.  The rail sits directly above the upper straight finger, so a box
# that starts to slip down is caught by the rail's underside.  It does not
# constrain the free joint or change contact rules.
# A 25 mm shelf gives the two nearly straight fingers enough bearing area to
# support the load with the whole hand.  With the earlier 15 mm shelf the
# fingers rolled off its outer edge during the 30 second hold.
BOX_RAIL_PROTRUSION = 0.025
BOX_RAIL_HEIGHT = 0.016
BOX_RAIL_CENTER_Z = 0.045
ROOM_HALF_EXTENT = 3.0
WALL_THICKNESS = 0.05
WALL_HEIGHT = 2.5
NAV_SCAN_GROUP = "2"


def ensure_ground_grid_material(root):
    asset = root.find("asset")
    if asset is None:
        asset = ET.Element("asset")
        world_index = next((index for index, child in enumerate(root) if child.tag == "worldbody"), len(root))
        root.insert(world_index, asset)
    if asset.find("material[@name='groundplane']") is not None:
        return "groundplane"
    if asset.find("texture[@name='ground_grid']") is None:
        ET.SubElement(asset, "texture", {
            "type": "2d",
            "name": "ground_grid",
            "builtin": "checker",
            "mark": "edge",
            "rgb1": "0.24 0.30 0.36",
            "rgb2": "0.14 0.20 0.26",
            "markrgb": "0.75 0.80 0.85",
            "width": "300",
            "height": "300",
        })
    ET.SubElement(asset, "material", {
        "name": "ground_grid",
        "texture": "ground_grid",
        "texuniform": "true",
        "texrepeat": "6 6",
        "reflectance": "0.15",
    })
    return "ground_grid"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("destination", type=Path)
    parser.add_argument("--scene", choices=("tabletop", "nav"), default="tabletop")
    parser.add_argument("--box-side-rails", action="store_true",
                        help="add two visible physical anti-slip rails to the box side faces")
    args = parser.parse_args()
    table_half_length = TABLETOP_TABLE_HALF_LENGTH if args.scene == "tabletop" else NAV_TABLE_HALF_LENGTH
    table_half_width = TABLETOP_TABLE_HALF_WIDTH if args.scene == "tabletop" else NAV_TABLE_HALF_WIDTH
    table_center_x = TABLETOP_TABLE_CENTER_X if args.scene == "tabletop" else NAV_TABLE_CENTER_X
    physical_tabletop = args.scene == "tabletop"
    box_length = RAILED_BOX_LENGTH if args.box_side_rails else BOX_LENGTH
    box_width = RAILED_BOX_WIDTH if args.box_side_rails else BOX_WIDTH
    box_height = RAILED_BOX_HEIGHT if args.box_side_rails else BOX_HEIGHT
    root = ET.parse(args.source).getroot()
    root.insert(1, ET.Element("option", {"timestep": "0.002", "gravity": "0 0 -9.81", "integrator": "implicitfast"}))
    default = ET.Element("default")
    ET.SubElement(default, "geom", {"friction": "1.4 0.02 0.001", "condim": "4", "solref": "0.008 1"})
    root.insert(2, default)
    world = root.find("worldbody")
    if world is None:
        raise RuntimeError("official model has no worldbody")
    ground_material = ensure_ground_grid_material(root)
    # This is a hand-contact and navigation bench, not a whole-body balance
    # controller. Keep the official floating base joint so the ROS/Nav2 adapter
    # can kinematically move the visible robot in MuJoCo, while gravity
    # compensation and explicit root-pose writes keep it from becoming a
    # balance-control problem. The box, table and ground remain ordinary
    # dynamic/contact bodies.
    pelvis = world.find("body[@name='pelvis']")
    if pelvis is None:
        raise RuntimeError("could not find pelvis in official model")
    for body in pelvis.iter("body"):
        body.set("gravcomp", "1")
    pelvis.set("gravcomp", "1")
    for joint in pelvis.iter("joint"):
        name = joint.get("name", "")
        if "_hand_" in name:
            joint.set("damping", "0.16")
        elif any(part in name for part in ("_hip_", "_knee_", "_ankle_")):
            joint.set("damping", "18")
        else:
            joint.set("damping", "3")
        if "_hand_" in name:
            joint.set("armature", "0.002")

    # In the tabletop scene, robot geoms collide with the table and pickup box
    # for the manipulation benchmark. In the navigation scene, the base is
    # kinematic and obstacles are enforced through lidar/costmaps instead of
    # MuJoCo contacts; otherwise the visual model can be dragged into a table
    # when testing impossible or very tight goals.
    for geom in pelvis.iter("geom"):
        geom.set("contype", "2" if physical_tabletop else "0")
        geom.set("conaffinity", "4" if physical_tabletop else "0")
        # DEX3 rubber pads are represented through Coulomb friction only.
        # There are no extra collision shapes or grasp constraints.
        parent = next((body for body in pelvis.iter("body") if geom in list(body)), None)
        if physical_tabletop and parent is not None and "_hand_" in parent.get("name", ""):
            geom.set("friction", "4.0 0.03 0.002")
    torso = pelvis.find(".//body[@name='torso_link']")
    if torso is None:
        raise RuntimeError("could not find torso_link in official model")
    # The official MJCF represents the head as a mesh rigidly attached to the
    # torso (rather than as a separate body).  This fixed child is the front
    # recess of that head mesh, so the sensor follows the head position and is
    # never attached to a hand or arm.
    head_camera_mount = ET.SubElement(torso, "body", {
        "name": "head_camera_mount", "pos": "0.075 0 0.420",
    })
    # MuJoCo cameras look along local -Z.  Aim the head-mounted D435i 40
    # degrees downward at the tabletop.
    ET.SubElement(head_camera_mount, "camera", {
        "name": "d435i", "pos": "0 0 0", "xyaxes": "0 -1 0 0.6427876 0 0.7660444",
        "fovy": "69",
    })

    ET.SubElement(world, "geom", {
        "name": "ground", "type": "plane", "size": "3 3 0.1",
        "contype": "8", "conaffinity": "4", "material": ground_material})
    wall_z = WALL_HEIGHT * 0.5
    ET.SubElement(world, "geom", {
        "name": "nav_wall_north", "type": "box",
        "pos": f"0 {ROOM_HALF_EXTENT} {wall_z}",
        "size": f"{ROOM_HALF_EXTENT + WALL_THICKNESS} {WALL_THICKNESS} {wall_z}",
        "contype": "0", "conaffinity": "0", "group": NAV_SCAN_GROUP, "rgba": "0.78 0.78 0.74 1"})
    ET.SubElement(world, "geom", {
        "name": "nav_wall_south", "type": "box",
        "pos": f"0 {-ROOM_HALF_EXTENT} {wall_z}",
        "size": f"{ROOM_HALF_EXTENT + WALL_THICKNESS} {WALL_THICKNESS} {wall_z}",
        "contype": "0", "conaffinity": "0", "group": NAV_SCAN_GROUP, "rgba": "0.78 0.78 0.74 1"})
    ET.SubElement(world, "geom", {
        "name": "nav_wall_east", "type": "box",
        "pos": f"{ROOM_HALF_EXTENT} 0 {wall_z}",
        "size": f"{WALL_THICKNESS} {ROOM_HALF_EXTENT + WALL_THICKNESS} {wall_z}",
        "contype": "0", "conaffinity": "0", "group": NAV_SCAN_GROUP, "rgba": "0.78 0.78 0.74 1"})
    ET.SubElement(world, "geom", {
        "name": "nav_wall_west", "type": "box",
        "pos": f"{-ROOM_HALF_EXTENT} 0 {wall_z}",
        "size": f"{WALL_THICKNESS} {ROOM_HALF_EXTENT + WALL_THICKNESS} {wall_z}",
        "contype": "0", "conaffinity": "0", "group": NAV_SCAN_GROUP, "rgba": "0.78 0.78 0.74 1"})
    table_center_z = TABLE_TOP_HEIGHT - TABLE_THICKNESS_HALF
    table = ET.SubElement(
        world, "body", {"name": "table", "pos": f"{table_center_x:.3f} 0 {table_center_z:.3f}"})
    ET.SubElement(
        table, "geom", {"name": "table_top", "type": "box",
                         "size": f"{table_half_length} {table_half_width} {TABLE_THICKNESS_HALF}",
                         "mass": "25",
                         "contype": "4" if physical_tabletop else "0",
                         "conaffinity": "6" if physical_tabletop else "0",
                         "group": NAV_SCAN_GROUP, "rgba": "0.45 0.25 0.10 1"})
    if physical_tabletop:
        # The side table is rotated about Z (default 90 deg, i.e. turned
        # sideways), so its axis-aligned Y extent changes.  Recompute the
        # effective half extents to keep it flush with the main table's right
        # edge (y = -TABLETOP_TABLE_HALF_WIDTH) without overlapping it.
        yaw = math.radians(SIDE_TABLE_YAW_DEG)
        cos_yaw, sin_yaw = abs(math.cos(yaw)), abs(math.sin(yaw))
        side_half_x = (SIDE_TABLE_HALF_LENGTH * cos_yaw
                       + SIDE_TABLE_HALF_WIDTH * sin_yaw)
        side_half_y = (SIDE_TABLE_HALF_LENGTH * sin_yaw
                       + SIDE_TABLE_HALF_WIDTH * cos_yaw)
        side_table_center_y = -(TABLETOP_TABLE_HALF_WIDTH + SIDE_TABLE_GAP + side_half_y)
        # Align the far edges of both tables on one line (x = main far edge)
        # so the side table extends backwards beside the robot and the two
        # tops form a clean right angle instead of crossing in the middle.
        main_far_edge_x = TABLETOP_TABLE_CENTER_X + TABLETOP_TABLE_HALF_LENGTH
        side_table_center_x = main_far_edge_x - side_half_x
        half_angle = yaw * 0.5
        side_table = ET.SubElement(world, "body", {
            "name": "side_table",
            "pos": f"{side_table_center_x:.3f} {side_table_center_y:.3f} {table_center_z:.3f}",
            "quat": f"{math.cos(half_angle):.6f} 0 0 {math.sin(half_angle):.6f}"})
        ET.SubElement(side_table, "geom", {
            "name": "side_table_top", "type": "box",
            "size": f"{SIDE_TABLE_HALF_LENGTH} {SIDE_TABLE_HALF_WIDTH} {TABLE_THICKNESS_HALF}",
            "mass": "25",
            "contype": "4", "conaffinity": "6",
            "group": NAV_SCAN_GROUP, "rgba": "0.52 0.30 0.13 1"})
    box_center_z = TABLE_TOP_HEIGHT + box_height * 0.5
    box = ET.SubElement(
        world, "body", {"name": "pickup_box", "pos": f"{table_center_x:.3f} 0 {box_center_z:.3f}"})
    ET.SubElement(box, "freejoint", {"name": "pickup_box_free"})
    ET.SubElement(
        box, "geom", {"name": "pickup_box_geom", "type": "box",
                       "size": f"{box_length * 0.5} {box_width * 0.5} {box_height * 0.5}",
                       # Two 2.5 g rails keep the complete free body at 0.25 kg.
                       "mass": "0.245" if args.box_side_rails else "0.25",
                       "contype": "4" if physical_tabletop else "0",
                       "conaffinity": "2" if physical_tabletop else "0",
                       "friction": "1.6 0.03 0.002", "group": NAV_SCAN_GROUP,
                       "rgba": "0.95 0.22 0.05 1"})
    if args.box_side_rails:
        for side, sign in (("left", 1.0), ("right", -1.0)):
            ET.SubElement(box, "geom", {
                "name": f"pickup_box_{side}_rail",
                "type": "box",
                "pos": (f"0 {sign * (box_width * 0.5 + BOX_RAIL_PROTRUSION * 0.5):.4f} "
                        f"{BOX_RAIL_CENTER_Z:.4f}"),
                "size": (f"{box_length * 0.5:.4f} {BOX_RAIL_PROTRUSION * 0.5:.4f} "
                         f"{BOX_RAIL_HEIGHT * 0.5:.4f}"),
                "mass": "0.0025",
                "contype": "4" if physical_tabletop else "0",
                "conaffinity": "2" if physical_tabletop else "0",
                "friction": "1.6 0.03 0.002",
                "group": NAV_SCAN_GROUP,
                "rgba": "0.72 0.10 0.02 1",
            })
    # The box is held by the palms and fingers.  The wrist links sit just
    # behind the palm; when the arm carries the box they can graze it and,
    # with the stiff default contacts, wedge and eject the free body.  Exclude
    # only the box<->wrist pairs; the grasp contacts on the palm and fingers
    # are untouched and the box stays a fully free body.
    contact = root.find("contact")
    if contact is None:
        contact = ET.Element("contact")
        root.insert(list(root).index(world) + 1, contact)
    for side in ("left", "right"):
        for part in ("wrist_roll", "wrist_pitch", "wrist_yaw"):
            name = f"{side}_{part}_link"
            if any(existing.get("body2") == name and existing.get("body1") == "pickup_box"
                   for existing in contact.findall("exclude")):
                continue
            ET.SubElement(contact, "exclude", {"body1": "pickup_box", "body2": name})
    ET.indent(root, space="  ")
    args.destination.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(args.destination, encoding="utf-8", xml_declaration=True)


if __name__ == "__main__":
    main()
