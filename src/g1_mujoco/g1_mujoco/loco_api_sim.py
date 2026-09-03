"""Simulation-side adapter for the Unitree G1 high-level locomotion API.

The real G1 exposes high-level locomotion through unitree_api Request/Response
messages on /g1/sim/api/sport/request and /g1/sim/api/sport/response.  MuJoCo cannot run the
closed firmware controller, but this node mirrors that wire contract and maps
the supported requests to a geometry_msgs/Twist command consumed by sim.py.
"""

import json
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from unitree_api.msg import Request, Response


API_GET_FSM_ID = 7001
API_GET_FSM_MODE = 7002
API_GET_BALANCE_MODE = 7003
API_GET_SWING_HEIGHT = 7004
API_GET_STAND_HEIGHT = 7005
API_SET_FSM_ID = 7101
API_SET_BALANCE_MODE = 7102
API_SET_SWING_HEIGHT = 7103
API_SET_STAND_HEIGHT = 7104
API_SET_VELOCITY = 7105
API_SET_ARM_TASK = 7106
API_SET_SPEED_MODE = 7107

# Convenience aliases from the generic Unitree sport API.  G1 LocoClient's
# high-level methods normally translate to the 7xxx IDs above, but accepting
# the 10xx aliases makes the simulator friendlier to existing examples.
API_SPORT_DAMP = 1001
API_SPORT_BALANCE_STAND = 1002
API_SPORT_STOP_MOVE = 1003
API_SPORT_STAND_UP = 1004
API_SPORT_STAND_DOWN = 1005
API_SPORT_RECOVERY_STAND = 1006
API_SPORT_MOVE = 1008

FSM_ZERO_TORQUE = 0
FSM_DAMP = 1
FSM_SQUAT = 2
FSM_SIT = 3
FSM_STAND_UP = 4
FSM_START = 500


class LocoApiSim(Node):
    def __init__(self):
        super().__init__("g1_loco_api_sim")
        self.request_topic = self.declare_parameter(
            "request_topic", "/g1/sim/api/sport/request").value
        self.response_topic = self.declare_parameter(
            "response_topic", "/g1/sim/api/sport/response").value
        self.cmd_vel_topic = self.declare_parameter(
            "cmd_vel_topic", "/cmd_vel").value
        self.publish_rate_hz = float(self.declare_parameter(
            "publish_rate_hz", 50.0).value)
        self.default_duration_s = float(self.declare_parameter(
            "default_duration_s", 1.0).value)
        self.max_linear_x = float(self.declare_parameter("max_linear_x", 0.5).value)
        self.max_linear_y = float(self.declare_parameter("max_linear_y", 0.3).value)
        self.max_angular_z = float(self.declare_parameter("max_angular_z", 0.8).value)
        self.require_start = bool(self.declare_parameter(
            "require_start", False).value)

        self.fsm_id = FSM_START if not self.require_start else FSM_DAMP
        self.fsm_mode = 0
        self.balance_mode = 0
        self.swing_height = 0.0
        self.stand_height = 0.0
        self.arm_task = 0
        self.speed_mode = 0
        self.active_twist = Twist()
        self.active_until = None
        self.stopped = True

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.response_pub = self.create_publisher(Response, self.response_topic, 10)
        self.request_sub = self.create_subscription(
            Request, self.request_topic, self.on_request, 10)
        self.timer = self.create_timer(
            1.0 / self.publish_rate_hz, self.publish_tick)
        self.get_logger().info(
            "G1 loco API simulator active: "
            f"{self.request_topic} -> {self.cmd_vel_topic}, "
            f"responses on {self.response_topic}, require_start={self.require_start}")

    def on_request(self, request):
        api_id = int(request.header.identity.api_id)
        try:
            code, payload = self.handle_request(api_id, request.parameter)
        except Exception as error:  # Defensive: malformed JSON should not kill sim.
            code = -1
            payload = {"ok": False, "api_id": api_id, "error": str(error)}
            self.get_logger().error(f"Failed to handle loco API {api_id}: {error}")

        if not bool(request.header.policy.noreply):
            self.publish_response(request, code, payload)

    def handle_request(self, api_id, parameter):
        if api_id == API_GET_FSM_ID:
            return 0, {"data": self.fsm_id}
        if api_id == API_GET_FSM_MODE:
            return 0, {"data": self.fsm_mode}
        if api_id == API_GET_BALANCE_MODE:
            return 0, {"data": self.balance_mode}
        if api_id == API_GET_SWING_HEIGHT:
            return 0, {"data": self.swing_height}
        if api_id == API_GET_STAND_HEIGHT:
            return 0, {"data": self.stand_height}

        if api_id == API_SET_FSM_ID:
            self.fsm_id = int(self.parse_parameter(parameter).get("data", self.fsm_id))
            if self.fsm_id in (FSM_ZERO_TORQUE, FSM_DAMP, FSM_SQUAT, FSM_SIT):
                self.stop_motion()
            return 0, {"ok": True, "fsm_id": self.fsm_id}
        if api_id == API_SET_BALANCE_MODE:
            self.balance_mode = int(self.parse_parameter(parameter).get("data", 0))
            return 0, {"ok": True, "balance_mode": self.balance_mode}
        if api_id == API_SET_SWING_HEIGHT:
            self.swing_height = float(self.parse_parameter(parameter).get("data", 0.0))
            return 0, {"ok": True, "swing_height": self.swing_height}
        if api_id == API_SET_STAND_HEIGHT:
            self.stand_height = float(self.parse_parameter(parameter).get("data", 0.0))
            return 0, {"ok": True, "stand_height": self.stand_height}
        if api_id == API_SET_ARM_TASK:
            self.arm_task = int(self.parse_parameter(parameter).get("data", 0))
            return 0, {"ok": True, "arm_task": self.arm_task}
        if api_id == API_SET_SPEED_MODE:
            self.speed_mode = int(self.parse_parameter(parameter).get("data", 0))
            return 0, {"ok": True, "speed_mode": self.speed_mode}
        if api_id == API_SET_VELOCITY:
            return self.handle_set_velocity(parameter)

        if api_id == API_SPORT_DAMP:
            self.fsm_id = FSM_DAMP
            self.stop_motion()
            return 0, {"ok": True, "fsm_id": self.fsm_id}
        if api_id == API_SPORT_BALANCE_STAND:
            self.balance_mode = 0
            return 0, {"ok": True, "balance_mode": self.balance_mode}
        if api_id == API_SPORT_STOP_MOVE:
            self.stop_motion()
            return 0, {"ok": True, "stopped": True}
        if api_id == API_SPORT_STAND_UP:
            self.fsm_id = FSM_STAND_UP
            return 0, {"ok": True, "fsm_id": self.fsm_id}
        if api_id == API_SPORT_STAND_DOWN:
            self.fsm_id = FSM_SQUAT
            self.stop_motion()
            return 0, {"ok": True, "fsm_id": self.fsm_id}
        if api_id == API_SPORT_RECOVERY_STAND:
            self.fsm_id = FSM_STAND_UP
            return 0, {"ok": True, "fsm_id": self.fsm_id}
        if api_id == API_SPORT_MOVE:
            return self.handle_sport_move(parameter)

        self.get_logger().warn(f"Unsupported simulated loco API id: {api_id}")
        return -1, {"ok": False, "api_id": api_id, "error": "unsupported_api"}

    def handle_set_velocity(self, parameter):
        data = self.parse_parameter(parameter)
        velocity = data.get("velocity", [0.0, 0.0, 0.0])
        if len(velocity) != 3:
            raise ValueError("velocity must contain [vx, vy, omega]")
        duration = float(data.get("duration", self.default_duration_s))
        return self.start_motion(float(velocity[0]), float(velocity[1]), float(velocity[2]), duration)

    def handle_sport_move(self, parameter):
        data = self.parse_parameter(parameter)
        duration = float(data.get("duration", self.default_duration_s))
        return self.start_motion(
            float(data.get("x", 0.0)),
            float(data.get("y", 0.0)),
            float(data.get("z", 0.0)),
            duration,
        )

    def start_motion(self, vx, vy, wz, duration):
        vx = self.clamp(vx, self.max_linear_x)
        vy = self.clamp(vy, self.max_linear_y)
        wz = self.clamp(wz, self.max_angular_z)
        duration = max(0.0, duration)
        if self.require_start and self.fsm_id != FSM_START:
            self.stop_motion()
            return 7302, {
                "ok": False,
                "error": "simulated_loco_not_started",
                "fsm_id": self.fsm_id,
            }
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = wz
        self.active_twist = twist
        self.active_until = self.get_clock().now().nanoseconds * 1e-9 + duration
        self.stopped = self.is_zero(vx, vy, wz) or duration <= 0.0
        self.cmd_pub.publish(twist)
        if self.stopped:
            self.stop_motion()
        return 0, {
            "ok": True,
            "velocity": [vx, vy, wz],
            "duration": duration,
            "fsm_id": self.fsm_id,
        }

    def stop_motion(self):
        self.active_twist = Twist()
        self.active_until = None
        if not self.stopped:
            self.cmd_pub.publish(self.active_twist)
        else:
            # Publish at least once: callers expect StopMove/zero velocity to
            # reach the simulated robot immediately, just like the real API.
            self.cmd_pub.publish(self.active_twist)
        self.stopped = True

    def publish_tick(self):
        if self.active_until is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now > self.active_until:
            self.stop_motion()
            return
        self.cmd_pub.publish(self.active_twist)

    def publish_response(self, request, code, payload):
        response = Response()
        response.header.identity.id = request.header.identity.id
        response.header.identity.api_id = request.header.identity.api_id
        response.header.status.code = int(code)
        response.data = json.dumps(payload, separators=(",", ":"))
        self.response_pub.publish(response)

    @staticmethod
    def parse_parameter(parameter):
        if not parameter:
            return {}
        parsed = json.loads(parameter)
        if not isinstance(parsed, dict):
            raise ValueError("parameter must be a JSON object")
        return parsed

    @staticmethod
    def clamp(value, limit):
        if not math.isfinite(value):
            return 0.0
        return max(-abs(limit), min(abs(limit), value))

    @staticmethod
    def is_zero(vx, vy, wz):
        return abs(vx) < 1e-6 and abs(vy) < 1e-6 and abs(wz) < 1e-6


def main():
    rclpy.init()
    node = LocoApiSim()
    try:
        rclpy.spin(node)
    finally:
        node.stop_motion()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
