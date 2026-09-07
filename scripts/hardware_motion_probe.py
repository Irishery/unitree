#!/usr/bin/env python3
"""Fixed, bounded first high-level walking probe for a supported physical G1."""

import os
import sys
import time

if __name__ == "__main__" and os.environ.get("G1_ALLOW_MOTION_TEST") != "YES":
    print(
        "Refusing to move. Set G1_ALLOW_MOTION_TEST=YES only after the gantry, "
        "clear-radius, Regular Mode, and controller-in-hand checks.",
        file=sys.stderr,
    )
    raise SystemExit(2)

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


TOPIC = "/g1/motion_cmd_vel"
SPEED_MPS = 0.05
MOVE_SECONDS = 0.5
RATE_HZ = 20.0


class MotionProbe(Node):
    def __init__(self):
        super().__init__("g1_hardware_motion_probe")
        command_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(Twist, TOPIC, command_qos)
        self.control_enabled = None
        self.subscription = self.create_subscription(
            Bool, "/g1/control_enabled", self._on_control_enabled, state_qos
        )
        self.disable_client = self.create_client(SetBool, "/g1/enable_control")

    def _on_control_enabled(self, message):
        self.control_enabled = message.data

    def publish_for(self, velocity, seconds):
        message = Twist()
        message.linear.x = velocity
        period = 1.0 / RATE_HZ
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.publisher.publish(message)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

    def stop_and_disarm(self):
        self.publish_for(0.0, 0.5)
        if self.disable_client.wait_for_service(timeout_sec=1.0):
            request = SetBool.Request()
            request.data = False
            future = self.disable_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)


def main():
    if len(sys.argv) != 1:
        print("This first probe takes no motion arguments.", file=sys.stderr)
        return 2

    rclpy.init()
    node = MotionProbe()
    try:
        deadline = time.monotonic() + 5.0
        while node.control_enabled is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.control_enabled is not True:
            print(
                "Refusing to move: /g1/control_enabled is not true. "
                "Arm it explicitly with /g1/enable_control first.",
                file=sys.stderr,
            )
            return 2
        if node.count_subscribers(TOPIC) < 1:
            print(f"Refusing to move: no bridge subscriber on {TOPIC}", file=sys.stderr)
            return 2

        print("Bounded probe: +0.05 m/s for 0.5 s, then stop and disarm.")
        print("Starting in 3 seconds; Ctrl-C aborts.")
        for remaining in (3, 2, 1):
            print(remaining, flush=True)
            time.sleep(1.0)
        node.publish_for(SPEED_MPS, MOVE_SECONDS)
        return 0
    except KeyboardInterrupt:
        print("Interrupted; sending stop.")
        return 130
    finally:
        try:
            node.stop_and_disarm()
            print("Stop sent; G1 software control disarmed.")
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
