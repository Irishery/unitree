#!/usr/bin/env python3
"""Add a contact-only tabletop scene to Unitree's official G1+DEx3 MJCF."""
import argparse
import xml.etree.ElementTree as ET
from pathlib import Path

TABLETOP_TABLE_HALF_LENGTH = 0.30
TABLETOP_TABLE_HALF_WIDTH = 0.70
TABLETOP_TABLE_CENTER_X = 0.15 + TABLETOP_TABLE_HALF_LENGTH
NAV_TABLE_HALF_LENGTH = 0.18
NAV_TABLE_HALF_WIDTH = 0.35
NAV_TABLE_CENTER_X = 0.55
TABLE_TOP_HEIGHT = 0.755
TABLE_THICKNESS_HALF = 0.04
BOX_LENGTH = 0.255
BOX_WIDTH = 0.370
BOX_HEIGHT = 0.090
ROOM_HALF_EXTENT = 3.0
WALL_THICKNESS = 0.05
WALL_HEIGHT = 2.5
NAV_SCAN_GROUP = "2"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("destination", type=Path)
    parser.add_argument("--scene", choices=("tabletop", "nav"), default="tabletop")
    args = parser.parse_args()
    table_half_length = TABLETOP_TABLE_HALF_LENGTH if args.scene == "tabletop" else NAV_TABLE_HALF_LENGTH
    table_half_width = TABLETOP_TABLE_HALF_WIDTH if args.scene == "tabletop" else NAV_TABLE_HALF_WIDTH
    table_center_x = TABLETOP_TABLE_CENTER_X if args.scene == "tabletop" else NAV_TABLE_CENTER_X
    physical_tabletop = args.scene == "tabletop"
    root = ET.parse(args.source).getroot()
    root.insert(1, ET.Element("option", {"timestep": "0.002", "gravity": "0 0 -9.81", "integrator": "implicitfast"}))
    default = ET.Element("default")
    ET.SubElement(default, "geom", {"friction": "1.4 0.02 0.001", "condim": "4", "solref": "0.008 1"})
    root.insert(2, default)
    world = root.find("worldbody")
    if world is None:
        raise RuntimeError("official model has no worldbody")
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

    ET.SubElement(world, "geom", {"name": "ground", "type": "plane", "size": "3 3 0.1", "contype": "8", "conaffinity": "4", "rgba": "0.25 0.25 0.25 1"})
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
    box_center_z = TABLE_TOP_HEIGHT + BOX_HEIGHT * 0.5
    box = ET.SubElement(
        world, "body", {"name": "pickup_box", "pos": f"{table_center_x:.3f} 0 {box_center_z:.3f}"})
    ET.SubElement(box, "freejoint", {"name": "pickup_box_free"})
    ET.SubElement(
        box, "geom", {"name": "pickup_box_geom", "type": "box",
                       "size": f"{BOX_LENGTH * 0.5} {BOX_WIDTH * 0.5} {BOX_HEIGHT * 0.5}",
                       "mass": "0.25",
                       "contype": "4" if physical_tabletop else "0",
                       "conaffinity": "2" if physical_tabletop else "0",
                       "friction": "1.6 0.03 0.002", "group": NAV_SCAN_GROUP,
                       "rgba": "0.95 0.22 0.05 1"})
    ET.indent(root, space="  ")
    args.destination.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(args.destination, encoding="utf-8", xml_declaration=True)


if __name__ == "__main__":
    main()
