"""Relay Nav2's local costmap voxel grid into PointCloud2 for RViz.

RViz2 (Jazzy) has no built-in display for ``nav2_msgs/msg/VoxelGrid``.
This node converts occupied voxels into a ``sensor_msgs/PointCloud2``
(voxel centre points) so the 3D obstacle grid can be visualised with the
regular PointCloud2 display.
"""

import struct

import rclpy
from nav2_msgs.msg import VoxelGrid
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField

POINT_STEP = 12  # three float32 fields: x, y, z


class VoxelGridRelay(Node):

    def __init__(self):
        super().__init__("voxel_grid_relay")
        self.declare_parameter("input_topic", "/local_costmap/voxel_grid")
        self.declare_parameter("output_topic", "/local_costmap/voxel_points")
        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self._pub = self.create_publisher(PointCloud2, output_topic, 5)
        self._sub = self.create_subscription(VoxelGrid, input_topic, self._on_grid, 5)
        self.get_logger().info(f"Relaying {input_topic} -> {output_topic}")

    def _on_grid(self, msg: VoxelGrid):
        # costmap_2d packs a whole (x, y) column into one uint32: index =
        # y * size_x + x (x fastest).  Each column holds two 16-bit banks:
        # bit b = voxel z = b UNKNOWN (set by default), bit 16 + b = voxel
        # z = b MARKED.  Raytraced-free voxels have both banks cleared.
        # Verified empirically against the nav scene: only columns with the
        # upper bit set line up with the table front.
        points = []
        res_x = msg.resolutions.x
        res_y = msg.resolutions.y
        res_z = msg.resolutions.z
        for index, column in enumerate(msg.data):
            marked = column >> 16
            if marked == 0:
                continue
            x = msg.origin.x + (index % msg.size_x + 0.5) * res_x
            y = msg.origin.y + (index // msg.size_x + 0.5) * res_y
            for bit in range(msg.size_z):
                if marked & (1 << bit):
                    z = msg.origin.z + (bit + 0.5) * res_z
                    points.append((x, y, z))

        cloud = PointCloud2()
        cloud.header = msg.header
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = POINT_STEP
        cloud.row_step = POINT_STEP * len(points)
        cloud.is_dense = True
        cloud.data = struct.pack(f"<{len(points) * 3}f", *[v for p in points for v in p])
        self._pub.publish(cloud)


def main():
    rclpy.init()
    node = VoxelGridRelay()
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
