#!/usr/bin/env python3
"""Convert Nav2 Twist commands into the first bounded physical-G1 profile."""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


def guard_command(vx, vy, wz, gait_speed, max_angular_speed):
    """Return a forward-only gait command and a rejection reason, if any."""
    values = (vx, vy, wz, gait_speed, max_angular_speed)
    if not all(math.isfinite(value) for value in values):
        return (0.0, 0.0, 0.0), "non-finite command"
    if gait_speed <= 0.0 or max_angular_speed < 0.0:
        return (0.0, 0.0, 0.0), "invalid guard limits"
    if vx < 0.0:
        return (0.0, 0.0, 0.0), "reverse command"
    if vx <= 1.0e-4:
        if abs(vy) > 1.0e-4 or abs(wz) > 1.0e-4:
            return (0.0, 0.0, 0.0), "sideways or in-place rotation command"
        return (0.0, 0.0, 0.0), ""

    angular = max(-max_angular_speed, min(max_angular_speed, wz))
    return (gait_speed, 0.0, angular), ""


class NavVelocityGuard(Node):
    def __init__(self):
        super().__init__("g1_nav_velocity_guard")
        self.declare_parameter("input_topic", "/g1/nav_cmd_vel_raw")
        self.declare_parameter("output_topic", "/g1/motion_cmd_vel")
        self.declare_parameter("gait_speed", 0.20)
        self.declare_parameter("max_angular_speed", 0.10)
        self.declare_parameter("command_timeout", 0.20)
        self.declare_parameter("publish_rate", 20.0)

        self._gait_speed = float(self.get_parameter("gait_speed").value)
        self._max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self._command_timeout = float(self.get_parameter("command_timeout").value)
        publish_rate = float(self.get_parameter("publish_rate").value)
        if (
            self._gait_speed <= 0.0
            or self._max_angular_speed < 0.0
            or self._command_timeout <= 0.0
            or publish_rate <= 0.0
        ):
            raise ValueError("invalid navigation velocity guard parameter")

        command_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(
            Twist, str(self.get_parameter("output_topic").value), command_qos
        )
        self._command_subscription = self.create_subscription(
            Twist,
            str(self.get_parameter("input_topic").value),
            self._on_command,
            command_qos,
        )
        self._control_subscription = self.create_subscription(
            Bool, "/g1/control_enabled", self._on_control, state_qos
        )
        self._control_enabled = False
        self._desired = (0.0, 0.0, 0.0)
        self._last_command_time = None
        self._zero_sent = False
        self._last_warning_time = 0.0
        self._timer = self.create_timer(1.0 / publish_rate, self._tick)
        self.get_logger().warning(
            f"Physical navigation guard ready but disarmed: forward-only "
            f"{self._gait_speed:.2f} m/s, |yaw| <= {self._max_angular_speed:.2f} rad/s"
        )

    def _publish(self, values):
        message = Twist()
        message.linear.x, message.linear.y, message.angular.z = values
        self._publisher.publish(message)

    def _on_control(self, message):
        enabled = bool(message.data)
        if enabled != self._control_enabled:
            self._desired = (0.0, 0.0, 0.0)
            self._last_command_time = None
            self._zero_sent = False
        self._control_enabled = enabled
        if not enabled and not self._zero_sent:
            self._publish((0.0, 0.0, 0.0))
            self._zero_sent = True

    def _on_command(self, message):
        values, reason = guard_command(
            message.linear.x,
            message.linear.y,
            message.angular.z,
            self._gait_speed,
            self._max_angular_speed,
        )
        self._last_command_time = time.monotonic()
        if reason:
            now = self._last_command_time
            if now - self._last_warning_time >= 1.0:
                self.get_logger().warning(f"Nav2 command stopped by guard: {reason}")
                self._last_warning_time = now
        self._desired = values
        self._zero_sent = False

    def _tick(self):
        now = time.monotonic()
        fresh = (
            self._last_command_time is not None
            and now - self._last_command_time <= self._command_timeout
        )
        if self._control_enabled and fresh:
            self._publish(self._desired)
            self._zero_sent = self._desired == (0.0, 0.0, 0.0)
        elif not self._zero_sent:
            self._publish((0.0, 0.0, 0.0))
            self._desired = (0.0, 0.0, 0.0)
            self._zero_sent = True


def main():
    rclpy.init()
    node = NavVelocityGuard()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
