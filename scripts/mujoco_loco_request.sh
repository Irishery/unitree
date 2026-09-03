#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  cat >&2 <<'USAGE'
Usage:
  ./scripts/mujoco_loco_request.sh start
  ./scripts/mujoco_loco_request.sh stop
  ./scripts/mujoco_loco_request.sh move VX VY WZ [DURATION_S]
  ./scripts/mujoco_loco_request.sh fsm FSM_ID

Examples:
  ./scripts/mujoco_loco_request.sh start
  ./scripts/mujoco_loco_request.sh move 0.15 0.0 0.0 1.0
  ./scripts/mujoco_loco_request.sh stop
USAGE
  exit 2
fi

container_name="${MUJOCO_CONTAINER:-unitree-g1-mujoco}"
command="$1"
shift

api_id=""
parameter="{}"

case "${command}" in
  start)
    api_id=7101
    parameter='{"data":500}'
    ;;
  damp)
    api_id=7101
    parameter='{"data":1}'
    ;;
  stand_up)
    api_id=7101
    parameter='{"data":4}'
    ;;
  stop)
    api_id=7105
    parameter='{"velocity":[0.0,0.0,0.0],"duration":0.0}'
    ;;
  fsm)
    if [[ $# -ne 1 ]]; then
      echo "Usage: $0 fsm FSM_ID" >&2
      exit 2
    fi
    api_id=7101
    parameter="{\"data\":$1}"
    ;;
  move)
    if [[ $# -lt 3 || $# -gt 4 ]]; then
      echo "Usage: $0 move VX VY WZ [DURATION_S]" >&2
      exit 2
    fi
    vx="$1"
    vy="$2"
    wz="$3"
    duration="${4:-1.0}"
    parameter="$(python3 - "$vx" "$vy" "$wz" "$duration" <<'PY'
import json
import sys

vx, vy, wz, duration = [float(value) for value in sys.argv[1:5]]
print(json.dumps({"velocity": [vx, vy, wz], "duration": duration}, separators=(",", ":")))
PY
)"
    api_id=7105
    ;;
  *)
    echo "Unknown command '${command}'" >&2
    exit 2
    ;;
esac

docker exec "${container_name}" bash -lc "
  source /opt/ros/jazzy/setup.bash
  source /ws/install/setup.bash
  python3 - '${api_id}' '${parameter}' <<'PY'
import json
import sys
import time

import rclpy
from rclpy.node import Node
from unitree_api.msg import Request, Response

api_id = int(sys.argv[1])
parameter = sys.argv[2]

class Client(Node):
    def __init__(self):
        super().__init__('mujoco_loco_request')
        self.done = False
        self.response = None
        self.request_id = time.time_ns()
        self.pub = self.create_publisher(Request, '/g1/sim/api/sport/request', 10)
        self.sub = self.create_subscription(Response, '/g1/sim/api/sport/response', self.on_response, 10)

    def on_response(self, msg):
        if msg.header.identity.id == self.request_id:
            self.response = msg
            self.done = True

rclpy.init()
node = Client()
deadline = time.time() + 5.0
while time.time() < deadline and node.pub.get_subscription_count() == 0:
    rclpy.spin_once(node, timeout_sec=0.1)

req = Request()
req.header.identity.id = node.request_id
req.header.identity.api_id = api_id
req.parameter = parameter
node.pub.publish(req)

deadline = time.time() + 5.0
while time.time() < deadline and not node.done:
    rclpy.spin_once(node, timeout_sec=0.1)

if node.response is None:
    print('No /g1/sim/api/sport/response received', file=sys.stderr)
    rclpy.shutdown()
    sys.exit(1)

payload = json.loads(node.response.data) if node.response.data else {}
print(json.dumps({
    'status_code': node.response.header.status.code,
    'api_id': node.response.header.identity.api_id,
    'data': payload,
}, indent=2, sort_keys=True))
rclpy.shutdown()
PY
"
