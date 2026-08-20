#!/usr/bin/env python3
"""Print a compact Nav2/MuJoCo navigation snapshot.

This is intentionally text-only so it can run inside the Docker container over
`docker exec` when RViz is already busy rendering.
"""

import argparse
import math
import time

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import LaserScan


def round_pair(point):
    return (round(float(point[0]), 3), round(float(point[1]), 3))


def summarize_costmap(name, grid):
    data = list(grid.data)
    width = int(grid.info.width)
    height = int(grid.info.height)
    resolution = float(grid.info.resolution)
    origin_x = float(grid.info.origin.position.x)
    origin_y = float(grid.info.origin.position.y)
    lethal = sum(1 for value in data if value >= 90)
    occupied = sum(1 for value in data if value > 50)
    unknown = sum(1 for value in data if value < 0)
    print(
        f"{name}: size={width}x{height} res={resolution:.3f} "
        f"origin=({origin_x:.2f},{origin_y:.2f}) "
        f"lethal={lethal} occupied={occupied} unknown={unknown}"
    )
    samples = [
        ("table_front", 0.95 - 0.18, 0.0),
        ("table_center", 0.95, 0.0),
        ("table_back", 0.95 + 0.18, 0.0),
        ("south_gap", 0.95, -0.80),
        ("north_gap", 0.95, 0.80),
    ]
    for label, x, y in samples:
        mx = int((x - origin_x) / resolution)
        my = int((y - origin_y) / resolution)
        if 0 <= mx < width and 0 <= my < height:
            value = grid.data[my * width + mx]
            print(f"  sample {label}@({x:.2f},{y:.2f}) cost={value}")
        else:
            print(f"  sample {label}@({x:.2f},{y:.2f}) outside")


def summarize_plan(path):
    poses = path.poses
    if not poses:
        print("/plan: empty")
        return
    points = [(pose.pose.position.x, pose.pose.position.y) for pose in poses]
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    print(
        "/plan: "
        f"n={len(points)} "
        f"first={[round_pair(point) for point in points[:8]]} "
        f"last={[round_pair(point) for point in points[-8:]]} "
        f"minmax=({min(xs):.2f},{max(xs):.2f},{min(ys):.2f},{max(ys):.2f})"
    )


def summarize_scan(scan):
    values = [(index, value) for index, value in enumerate(scan.ranges) if math.isfinite(value)]
    closest = sorted(values, key=lambda item: item[1])[:20]
    closest_fmt = [
        (index, round(float(value), 3), round(float(scan.angle_min + index * scan.angle_increment), 3))
        for index, value in closest
    ]
    front_index = len(scan.ranges) // 2
    front_range = scan.ranges[front_index] if scan.ranges else float("nan")
    print(f"/scan: finite={len(values)} closest={closest_fmt}")
    print(f"/scan: front_index={front_index} front_range={front_range}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=8.0)
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("mujoco_nav_debug")
    got = {}
    node.create_subscription(Path, "/plan", lambda msg: got.__setitem__("plan", msg), 10)
    node.create_subscription(
        OccupancyGrid,
        "/global_costmap/costmap",
        lambda msg: got.__setitem__("global_costmap", msg),
        10,
    )
    node.create_subscription(
        OccupancyGrid,
        "/local_costmap/costmap",
        lambda msg: got.__setitem__("local_costmap", msg),
        10,
    )
    node.create_subscription(Odometry, "/odom", lambda msg: got.__setitem__("odom", msg), 10)
    node.create_subscription(LaserScan, "/scan", lambda msg: got.__setitem__("scan", msg), 10)

    deadline = time.time() + args.timeout
    while time.time() < deadline and len(got) < 5:
        rclpy.spin_once(node, timeout_sec=0.1)

    print("received:", ", ".join(sorted(got)) or "nothing")
    odom = got.get("odom")
    if odom is not None:
        pose = odom.pose.pose
        q = pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        print(f"/odom: x={pose.position.x:.3f} y={pose.position.y:.3f} yaw={yaw:.3f}")
    if "plan" in got:
        summarize_plan(got["plan"])
    if "scan" in got:
        summarize_scan(got["scan"])
    if "global_costmap" in got:
        summarize_costmap("/global_costmap/costmap", got["global_costmap"])
    if "local_costmap" in got:
        summarize_costmap("/local_costmap/costmap", got["local_costmap"])

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
