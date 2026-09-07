#!/usr/bin/env python3
"""Fixed, bounded first high-level walking probe for a supported physical G1."""

import os
import signal
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
from rclpy.signals import SignalHandlerOptions
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
            rclpy.spin_once(self, timeout_sec=0.0)
            if velocity != 0.0 and self.control_enabled is not True:
                raise RuntimeError("Software control was disabled during the probe")
            self.publisher.publish(message)
            time.sleep(period)

    def stop_and_disarm(self):
        try:
            self.publish_for(0.0, 0.5)
            print("Zero velocity messages sent; physical stop is not confirmed.")
        except Exception as error:
            print(f"Zero velocity publication failed: {error}", file=sys.stderr)
        # Still attempt disarming if the separate velocity publication failed.
        try:
            if not self.disable_client.wait_for_service(timeout_sec=1.0):
                print("Disarm service unavailable", file=sys.stderr)
                return False
            request = SetBool.Request()
            request.data = False
            self.control_enabled = None  # Require a new state, not cached false.
            future = self.disable_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if not future.done():
                print("Disarm service response timed out", file=sys.stderr)
                return False
            response = future.result()
            if response is None or not response.success:
                print(f"Disarm rejected: {getattr(response, 'message', 'empty response')}", file=sys.stderr)
                return False
            deadline = time.monotonic() + 2.0
            while self.control_enabled is not False and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
            if self.control_enabled is not False:
                print("No fresh /g1/control_enabled=false confirmation", file=sys.stderr)
                return False
            return True
        except Exception as error:
            print(f"Disarm failed: {error}", file=sys.stderr)
            return False


def interrupt_probe(_signum, _frame):
    # Keep ROS alive long enough to attempt the bounded stop/disarm sequence.
    raise KeyboardInterrupt


def main():
    if len(sys.argv) != 1:
        print("This first probe takes no motion arguments.", file=sys.stderr)
        return 2

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    previous_signals = {
        sig: signal.signal(sig, interrupt_probe) for sig in (signal.SIGINT, signal.SIGTERM)
    }
    node = MotionProbe()
    exit_code = 0
    try:
        deadline = time.monotonic() + 5.0
        while node.control_enabled is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.control_enabled is not True:
            raise RuntimeError(
                "Refusing to move: /g1/control_enabled is not true. "
                "Arm it explicitly with /g1/enable_control first."
            )
        if node.count_subscribers(TOPIC) < 1:
            raise RuntimeError(f"Refusing to move: no bridge subscriber on {TOPIC}")

        print("Bounded probe: +0.05 m/s for 0.5 s, then stop and disarm.")
        print("Starting in 3 seconds; Ctrl-C aborts.")
        for remaining in (3, 2, 1):
            print(remaining, flush=True)
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                if node.control_enabled is not True:
                    raise RuntimeError("Software control was disabled during countdown")
        node.publish_for(SPEED_MPS, MOVE_SECONDS)
        print("Bounded velocity publication completed; robot movement is not verified.")
    except KeyboardInterrupt:
        print("Interrupted; sending stop.")
        exit_code = 130
    except Exception as error:
        print(f"Probe failed: {error}", file=sys.stderr)
        exit_code = 2
    finally:
        # Do not let a repeated Ctrl-C interrupt this short best-effort cleanup.
        for sig in previous_signals:
            signal.signal(sig, signal.SIG_IGN)
        try:
            if node.stop_and_disarm():
                print("G1 software control disarmed: service success and state=false confirmed.")
            else:
                print(
                    "ERROR: software disarm NOT confirmed. Use the official controller "
                    "to stop the robot; do not repeat the probe.", file=sys.stderr,
                )
                exit_code = 3
        finally:
            node.destroy_node()
            rclpy.shutdown()
            for sig, handler in previous_signals.items():
                signal.signal(sig, handler)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
