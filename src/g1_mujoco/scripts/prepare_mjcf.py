#!/usr/bin/env python3
"""Add the stationary bottle tabletop scene to Unitree's official G1+DEX3 MJCF."""
import argparse
import math
import xml.etree.ElementTree as ET
from pathlib import Path

TABLETOP_TABLE_HALF_LENGTH = 0.30
TABLETOP_TABLE_HALF_WIDTH = 0.70
TABLETOP_TABLE_CENTER_X = 0.15 + TABLETOP_TABLE_HALF_LENGTH
NAV_TABLE_HALF_LENGTH = 0.18
NAV_TABLE_HALF_WIDTH = 0.35
NAV_TABLE_CENTER_X = 0.95
TABLE_TOP_HEIGHT = 0.755
TABLE_THICKNESS_HALF = 0.04

# Upright bottle on the table.  A lathed mesh gives the body a real shoulder
# and neck instead of a plain cylinder; a separate cap and label ring make the
# silhouette read as a bottle.  The body is red for the RGB-D colour mask.
BOTTLE_BODY_RADIUS = 0.031
BOTTLE_HEIGHT = 0.235
BOTTLE_CAP_RADIUS = 0.014
BOTTLE_CAP_HEIGHT = 0.015
BOTTLE_BODY_MASS = 0.400
BOTTLE_BASE_RADIUS = 0.060
BOTTLE_BASE_HEIGHT = 0.012
BOTTLE_BASE_MASS = 0.100
BOTTLE_CAP_MASS = 0.003
BOTTLE_FRICTION = "3.0 0.05 0.005"
BOTTLE_LABEL_RADIUS = 0.0312
BOTTLE_LABEL_HALF_HEIGHT = 0.030
BOTTLE_LABEL_Z = 0.000
# Profile in the bottle body frame: main body (r=0.05) from z=-0.11 to 0.05,
# smooth shoulder up to the 15 mm neck, then a small lip at z=0.172.
BOTTLE_PROFILE = (
    (0.0310, -0.1175),
    (0.0310, 0.020),
    (0.0305, 0.032),
    (0.0290, 0.043),
    (0.0260, 0.053),
    (0.0220, 0.062),
    (0.0180, 0.070),
    (0.0140, 0.077),
    (0.0115, 0.083),
    (0.0110, 0.1025),
    (0.0130, 0.1025),
    (0.0130, 0.107),
)

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


def bottle_mesh_arrays(segments=32):
    """Return a closed lathed bottle mesh as (vertices, triangle faces)."""
    vertices = []
    rings = []
    for radius, z in BOTTLE_PROFILE:
        rings.append(len(vertices))
        for index in range(segments):
            angle = 2.0 * math.pi * index / segments
            vertices.append((radius * math.cos(angle),
                             radius * math.sin(angle), z))
    faces = []
    for ring in range(len(BOTTLE_PROFILE) - 1):
        for index in range(segments):
            a = rings[ring] + index
            b = rings[ring] + (index + 1) % segments
            c = rings[ring + 1] + index
            d = rings[ring + 1] + (index + 1) % segments
            faces.append((a, b, d))
            faces.append((a, d, c))
    bottom_centre = len(vertices)
    vertices.append((0.0, 0.0, BOTTLE_PROFILE[0][1]))
    top_centre = len(vertices)
    vertices.append((0.0, 0.0, BOTTLE_PROFILE[-1][1]))
    for index in range(segments):
        a = rings[0] + index
        b = rings[0] + (index + 1) % segments
        faces.append((bottom_centre, b, a))
        c = rings[-1] + index
        d = rings[-1] + (index + 1) % segments
        faces.append((top_centre, c, d))
    return vertices, faces


def add_bottle_mesh(root):
    asset = root.find("asset")
    if asset is None:
        raise RuntimeError("asset section is missing")
    vertices, faces = bottle_mesh_arrays()
    vertex_text = " ".join(f"{value:.6f}" for vertex in vertices for value in vertex)
    face_text = " ".join(str(index) for face in faces for index in face)
    ET.SubElement(asset, "mesh", {
        "name": "bottle_mesh", "vertex": vertex_text, "face": face_text,
        "smoothnormal": "true"})


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("destination", type=Path)
    parser.add_argument("--scene", choices=("tabletop", "nav"), default="tabletop")
    args = parser.parse_args()
    table_half_length = (TABLETOP_TABLE_HALF_LENGTH if args.scene == "tabletop"
                         else NAV_TABLE_HALF_LENGTH)
    table_half_width = (TABLETOP_TABLE_HALF_WIDTH if args.scene == "tabletop"
                        else NAV_TABLE_HALF_WIDTH)
    table_center_x = (TABLETOP_TABLE_CENTER_X if args.scene == "tabletop"
                      else NAV_TABLE_CENTER_X)
    physical_tabletop = args.scene == "tabletop"
    root = ET.parse(args.source).getroot()
    root.insert(1, ET.Element("option", {
        "timestep": "0.002", "gravity": "0 0 -9.81", "integrator": "implicitfast"}))
    default = ET.Element("default")
    ET.SubElement(default, "geom", {
        "friction": "1.4 0.02 0.001", "condim": "4", "solref": "0.008 1"})
    root.insert(2, default)
    world = root.find("worldbody")
    if world is None:
        raise RuntimeError("official model has no worldbody")
    ground_material = ensure_ground_grid_material(root)
    if physical_tabletop:
        add_bottle_mesh(root)
    # The robot is stationary at the table: gravity compensation and a fixed
    # floating-base pose keep the visual model standing without a balance
    # controller.  The bottle, table and ground remain ordinary dynamic bodies.
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
            joint.set("armature", "0.002")
        elif any(part in name for part in ("_hip_", "_knee_", "_ankle_")):
            joint.set("damping", "18")
        else:
            joint.set("damping", "3")

    # In the tabletop scene, robot geoms collide with the table and bottle for
    # the manipulation benchmark.  In the navigation scene, the base is
    # kinematic and obstacles are enforced through lidar/costmaps instead of
    # MuJoCo contacts.
    for geom in pelvis.iter("geom"):
        if geom.get("group") == "1" and geom.get("density") == "0":
            geom.set("contype", "0")
            geom.set("conaffinity", "0")
            continue
        geom.set("contype", "2" if physical_tabletop else "0")
        geom.set("conaffinity", "4" if physical_tabletop else "0")
        parent = next((body for body in pelvis.iter("body") if geom in list(body)), None)
        if physical_tabletop and parent is not None and "_hand_" in parent.get("name", ""):
            geom.set("friction", "6.0 0.05 0.005")
            geom.set("condim", "6")
    torso = pelvis.find(".//body[@name='torso_link']")
    if torso is None:
        raise RuntimeError("could not find torso_link in official model")
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
    for name, pos, size in (
        ("nav_wall_north", f"0 {ROOM_HALF_EXTENT} {wall_z}",
         f"{ROOM_HALF_EXTENT + WALL_THICKNESS} {WALL_THICKNESS} {wall_z}"),
        ("nav_wall_south", f"0 {-ROOM_HALF_EXTENT} {wall_z}",
         f"{ROOM_HALF_EXTENT + WALL_THICKNESS} {WALL_THICKNESS} {wall_z}"),
        ("nav_wall_east", f"{ROOM_HALF_EXTENT} 0 {wall_z}",
         f"{WALL_THICKNESS} {ROOM_HALF_EXTENT + WALL_THICKNESS} {wall_z}"),
        ("nav_wall_west", f"{-ROOM_HALF_EXTENT} 0 {wall_z}",
         f"{WALL_THICKNESS} {ROOM_HALF_EXTENT + WALL_THICKNESS} {wall_z}"),
    ):
        ET.SubElement(world, "geom", {
            "name": name, "type": "box", "pos": pos, "size": size,
            "contype": "0", "conaffinity": "0", "group": NAV_SCAN_GROUP,
            "rgba": "0.78 0.78 0.74 1"})

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
        bottle_center_z = TABLE_TOP_HEIGHT + BOTTLE_HEIGHT * 0.5
        # Free body with a hidden wide collision base.  The visible mesh keeps
        # the 0.5 l bottle silhouette; the wider support polygon keeps it
        # upright when the fingers slip during a contact grasp attempt.
        bottle = ET.SubElement(
            world, "body", {"name": "pickup_bottle",
                            "pos": f"{table_center_x:.3f} 0 {bottle_center_z:.3f}"})
        ET.SubElement(bottle, "freejoint", {"name": "pickup_bottle_free"})
        # Invisible collision cylinder; the visual lathe mesh is lighter than
        # the real bottle and would make the contact model unnecessarily slow.
        ET.SubElement(bottle, "geom", {
            "name": "bottle_collision", "type": "cylinder",
            "pos": "0 0 -0.045",
            "size": f"{BOTTLE_BODY_RADIUS} {0.0725}",
            "mass": str(BOTTLE_BODY_MASS),
            "contype": "4", "conaffinity": "2",
            "friction": BOTTLE_FRICTION, "condim": "6",
            "group": "0", "rgba": "0.92 0.12 0.05 1"})
        # Wider bottom ring: a 0.5 l bottle has a small but real base flare,
        # and it keeps the free bottle from tipping when the fingers slip.
        ET.SubElement(bottle, "geom", {
            "name": "bottle_base_collision", "type": "cylinder",
            "pos": "0 0 -0.1115",
            "size": f"{BOTTLE_BASE_RADIUS} {BOTTLE_BASE_HEIGHT * 0.5}",
            "mass": str(BOTTLE_BASE_MASS),
            "contype": "4", "conaffinity": "2",
            "friction": BOTTLE_FRICTION, "condim": "6",
            "group": "3", "rgba": "0.92 0.12 0.05 0"})
        ET.SubElement(bottle, "geom", {
            "name": "bottle_body_geom", "type": "mesh", "mesh": "bottle_mesh",
            "mass": "0",
            "contype": "0", "conaffinity": "0",
            "friction": BOTTLE_FRICTION, "condim": "6",
            "group": NAV_SCAN_GROUP, "rgba": "0.92 0.12 0.05 1"})
        ET.SubElement(bottle, "geom", {
            "name": "bottle_label_geom", "type": "cylinder",
            "pos": f"0 0 {BOTTLE_LABEL_Z:.4f}",
            "size": f"{BOTTLE_LABEL_RADIUS} {BOTTLE_LABEL_HALF_HEIGHT}",
            "mass": "0.001",
            "contype": "0", "conaffinity": "0",
            "group": NAV_SCAN_GROUP, "rgba": "0.92 0.92 0.88 1"})
        ET.SubElement(bottle, "geom", {
            "name": "bottle_cap_geom", "type": "cylinder",
            "pos": "0 0 0.110",
            "size": f"{BOTTLE_CAP_RADIUS} {BOTTLE_CAP_HEIGHT * 0.5}",
            "mass": str(BOTTLE_CAP_MASS),
            "contype": "0", "conaffinity": "0",
            "friction": BOTTLE_FRICTION, "condim": "6",
            "group": NAV_SCAN_GROUP, "rgba": "0.10 0.30 0.85 1"})
    ET.indent(root, space="  ")
    args.destination.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(args.destination, encoding="utf-8", xml_declaration=True)


if __name__ == "__main__":
    main()
