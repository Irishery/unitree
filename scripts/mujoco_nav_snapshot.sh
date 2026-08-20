#!/usr/bin/env bash
set -euo pipefail

container="${MUJOCO_CONTAINER:-unitree-g1-mujoco}"
out="${1:-debug/mujoco_nav_snapshot.png}"
out_dir="$(dirname "${out}")"

mkdir -p "${out_dir}"

docker exec "${container}" bash -lc '
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
python3 - <<'"'"'PY'"'"'
import math
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class Snapshot(Node):
    def __init__(self):
        super().__init__("mujoco_nav_snapshot")
        transient = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        sensor = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        regular = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.scan = None
        self.odom = None
        self.plan = None
        self.global_costmap = None
        self.local_costmap = None
        self.goal = None
        self.create_subscription(LaserScan, "/scan", self._scan, sensor)
        self.create_subscription(Odometry, "/odom", self._odom, regular)
        self.create_subscription(Path, "/plan", self._plan, regular)
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap", self._global_costmap, transient)
        self.create_subscription(OccupancyGrid, "/local_costmap/costmap", self._local_costmap, transient)
        self.create_subscription(PoseStamped, "/goal_pose", self._goal, regular)

    def _scan(self, msg):
        self.scan = msg

    def _odom(self, msg):
        self.odom = msg

    def _plan(self, msg):
        self.plan = msg

    def _global_costmap(self, msg):
        self.global_costmap = msg

    def _local_costmap(self, msg):
        self.local_costmap = msg

    def _goal(self, msg):
        self.goal = msg


def draw_costmap(ax, grid, title, cmap, alpha):
    if grid is None:
        return False
    width = grid.info.width
    height = grid.info.height
    if width == 0 or height == 0:
        return False
    data = np.asarray(grid.data, dtype=np.int16).reshape((height, width))
    masked = np.ma.masked_where(data < 1, data)
    origin = grid.info.origin.position
    res = grid.info.resolution
    extent = [origin.x, origin.x + width * res, origin.y, origin.y + height * res]
    ax.imshow(masked, origin="lower", extent=extent, cmap=cmap, alpha=alpha, vmin=1, vmax=100)
    ax.text(extent[0], extent[3], title, fontsize=8, color="black", va="top")
    return True


def draw_robot_and_scan(ax, odom, scan):
    if odom is None:
        return False
    pos = odom.pose.pose.position
    yaw = yaw_from_quat(odom.pose.pose.orientation)
    ax.scatter([pos.x], [pos.y], c="tab:blue", s=80, label="robot /odom", zorder=5)
    ax.arrow(pos.x, pos.y, 0.35 * math.cos(yaw), 0.35 * math.sin(yaw),
             color="tab:blue", width=0.015, length_includes_head=True, zorder=5)
    if scan is not None:
        xs = []
        ys = []
        angle = scan.angle_min
        for r in scan.ranges:
            if math.isfinite(r) and scan.range_min <= r <= scan.range_max:
                xs.append(pos.x + r * math.cos(yaw + angle))
                ys.append(pos.y + r * math.sin(yaw + angle))
            angle += scan.angle_increment
        if xs:
            ax.scatter(xs, ys, c="tab:red", s=2, alpha=0.55, label="/scan")
    return True


def draw_plan(ax, plan):
    if plan is None or not plan.poses:
        return False
    xs = [p.pose.position.x for p in plan.poses]
    ys = [p.pose.position.y for p in plan.poses]
    ax.plot(xs, ys, color="limegreen", linewidth=2.0, label="/plan")
    return True


def draw_goal(ax, goal):
    if goal is None:
        return False
    p = goal.pose.position
    ax.scatter([p.x], [p.y], marker="*", c="gold", edgecolor="black", s=180, label="/goal_pose", zorder=6)
    return True


def main():
    rclpy.init()
    node = Snapshot()
    deadline = time.monotonic() + 8.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.odom is not None and node.scan is not None and (
            node.global_costmap is not None or node.local_costmap is not None
        ):
            break

    fig, ax = plt.subplots(figsize=(9, 9), dpi=130)
    got = []
    if draw_costmap(ax, node.global_costmap, "global costmap", "Greys", 0.55):
        got.append("/global_costmap/costmap")
    if draw_costmap(ax, node.local_costmap, "local costmap", "Oranges", 0.45):
        got.append("/local_costmap/costmap")
    if draw_plan(ax, node.plan):
        got.append("/plan")
    if draw_robot_and_scan(ax, node.odom, node.scan):
        got.append("/odom")
        if node.scan is not None:
            got.append("/scan")
    if draw_goal(ax, node.goal):
        got.append("/goal_pose")

    ax.set_title("MuJoCo Nav2 snapshot: " + (", ".join(got) if got else "no topics received"))
    ax.set_xlabel("x, m")
    ax.set_ylabel("y, m")
    ax.grid(True, alpha=0.35)
    ax.set_aspect("equal", adjustable="box")
    if ax.get_legend_handles_labels()[0]:
        ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig("/tmp/mujoco_nav_snapshot.png")
    print("wrote /tmp/mujoco_nav_snapshot.png")
    print("received:", ", ".join(got) if got else "none")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
PY
'

docker cp "${container}:/tmp/mujoco_nav_snapshot.png" "${out}"
echo "Wrote ${out}"
