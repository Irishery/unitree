"""Bridge Nav2-style cmd_vel into the simulated Unitree locomotion API.

This is intentionally simulation-only.  On the real robot, g1_bridge publishes
the same /api/sport/request SetVelocity messages and guards them with real
lowstate/diagnostics/watchdogs.  In MuJoCo we use this lightweight bridge to
exercise the full command chain:

  Nav2 /cmd_vel -> /api/sport/request -> g1_loco_api_sim -> sim command topic.
"""

import json
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from unitree_api.msg import Request


API_SET_VELOCITY = 7105


class CmdVelLocoBridge(Node):
    def __init__(self):
        super().__init__("g1_cmd_vel_loco_bridge")
        self.cmd_vel_topic = self.declare_parameter("cmd_vel_topic", "/cmd_vel").value
        self.request_topic = self.declare_parameter(
            "request_topic", "/api/sport/request").value
        self.publish_rate_hz = float(self.declare_parameter(
            "publish_rate_hz", 20.0).value)
        self.cmd_timeout_s = float(self.declare_parameter(
            "cmd_timeout_s", 0.25).value)
        self.command_duration_s = float(self.declare_parameter(
            "command_duration_s", 0.2).value)
        self.max_linear_x = float(self.declare_parameter("max_linear_x", 0.5).value)
        self.max_linear_y = float(self.declare_parameter("max_linear_y", 0.3).value)
        self.max_angular_z = float(self.declare_parameter("max_angular_z", 0.8).value)
        self.start_enabled = bool(self.declare_parameter("start_enabled", True).value)
        self.noreply = bool(self.declare_parameter("noreply", False).value)

        self.enabled = self.start_enabled
        self.desired_vx = 0.0
        self.desired_vy = 0.0
        self.desired_wz = 0.0
        self.have_cmd = False
        self.watchdog_stopped = False
        self.last_cmd_time = self.get_clock().now().nanoseconds * 1e-9 - 10.0

        self.request_pub = self.create_publisher(Request, self.request_topic, 10)
        self.cmd_sub = self.create_subscription(Twist, self.cmd_vel_topic, self.on_cmd_vel, 10)
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.tick)
        self.get_logger().info(
            "G1 cmd_vel -> loco API bridge active: "
            f"{self.cmd_vel_topic} -> {self.request_topic}, enabled={self.enabled}")

    def on_cmd_vel(self, msg):
        self.desired_vx = self.clamp(msg.linear.x, self.max_linear_x)
        self.desired_vy = self.clamp(msg.linear.y, self.max_linear_y)
        self.desired_wz = self.clamp(msg.angular.z, self.max_angular_z)
        self.have_cmd = True
        self.watchdog_stopped = False
        self.last_cmd_time = self.get_clock().now().nanoseconds * 1e-9

    def tick(self):
        if not self.enabled:
            return
        age = self.get_clock().now().nanoseconds * 1e-9 - self.last_cmd_time
        if not self.have_cmd or age > self.cmd_timeout_s:
            if not self.watchdog_stopped:
                self.publish_velocity(0.0, 0.0, 0.0, 0.0)
                self.watchdog_stopped = True
                self.get_logger().warn("cmd_vel watchdog sent simulated Unitree stop")
            return
        self.publish_velocity(
            self.desired_vx,
            self.desired_vy,
            self.desired_wz,
            self.command_duration_s,
        )

    def publish_velocity(self, vx, vy, wz, duration):
        request = Request()
        request.header.identity.id = time.time_ns()
        request.header.identity.api_id = API_SET_VELOCITY
        request.header.policy.noreply = self.noreply
        request.parameter = json.dumps({
            "velocity": [float(vx), float(vy), float(wz)],
            "duration": float(duration),
        }, separators=(",", ":"))
        self.request_pub.publish(request)

    @staticmethod
    def clamp(value, limit):
        return max(-abs(limit), min(abs(limit), float(value)))


def main():
    rclpy.init()
    node = CmdVelLocoBridge()
    try:
        rclpy.spin(node)
    finally:
        node.publish_velocity(0.0, 0.0, 0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
