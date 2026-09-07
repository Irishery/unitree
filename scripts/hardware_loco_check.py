#!/usr/bin/env python3
"""Read-only locomotion queries and passive request observation. No motion APIs."""

from collections import Counter, defaultdict
from datetime import datetime
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time
import traceback


READ_QUERIES = {7001: "GetFsmId", 7002: "GetFsmMode", 7003: "GetBalanceMode"}
OBSERVE_SECONDS = 10.0


def read_request(message_type, api_id, request_id):
    if api_id not in READ_QUERIES:
        raise ValueError("Only read-only locomotion APIs 7001/7002/7003 are allowed")
    message = message_type()
    message.header.identity.api_id = api_id
    message.header.identity.id = request_id
    message.header.policy.noreply = False
    message.parameter = "{}"
    return message


def sdk_inventory(emit):
    """Inspect files/metadata only; never import or execute vendor SDK code."""
    candidates = set()
    for package in ("unitree_sdk2py", "unitree_sdk2_python"):
        try:
            dist = importlib.metadata.distribution(package)
            emit("python_distribution", name=package, version=dist.version,
                 root=str(dist.locate_file("")))
            for relative in dist.files or []:
                if str(relative).endswith(("g1_loco_client.py", "g1_loco_api.py")):
                    candidates.add(Path(dist.locate_file(relative)))
        except importlib.metadata.PackageNotFoundError:
            pass
    roots = [Path.home(), Path.home() / "g1_dev"]
    for root in roots:
        for repo in ("unitree_sdk2_python", "unitree_sdk2", "unitree_ros2"):
            checkout = root / repo
            if not checkout.is_dir():
                continue
            try:
                commit = subprocess.run(
                    ["git", "-C", str(checkout), "rev-parse", "HEAD"],
                    capture_output=True, text=True, timeout=3, check=False,
                )
                emit("sdk_checkout", path=str(checkout), commit=commit.stdout.strip())
            except (OSError, subprocess.TimeoutExpired) as error:
                emit("sdk_checkout_error", path=str(checkout), error=str(error))
            for relative in (
                "unitree_sdk2py/g1/loco/g1_loco_client.py",
                "unitree_sdk2py/g1/loco/g1_loco_api.py",
                "include/unitree/robot/g1/loco/g1_loco_client.hpp",
                "include/unitree/robot/g1/loco/g1_loco_api.hpp",
                "example/src/include/g1/g1_loco_client.hpp",
            ):
                candidates.add(checkout / relative)
    # Editable installations may only have a sys.path entry, not metadata.
    for entry in sys.path:
        for filename in ("g1_loco_client.py", "g1_loco_api.py"):
            candidates.add(Path(entry) / "unitree_sdk2py/g1/loco" / filename)
    relevant = re.compile(
        r"GetFsm|GetBalance|SetVelocity|SwitchToUser|SwitchToInternal|"
        r"LOCO_API_VERSION|SWITCH_TO_|GET_FSM|GET_BALANCE|SET_VELOCITY"
    )
    found = set()
    for path in sorted(candidates):
        if not path.is_file() or path.resolve() in found:
            continue
        found.add(path.resolve())
        try:
            raw = path.read_bytes()
            lines = raw.decode("utf-8", errors="replace").splitlines()
            selected = set()
            for i, line in enumerate(lines):
                if relevant.search(line):
                    selected.update(range(max(0, i - 1), min(len(lines), i + 8)))
            excerpts = [{"line": i + 1, "text": lines[i].strip()[:240]}
                        for i in sorted(selected)]
            emit("sdk_source", path=str(path.resolve()),
                 sha256=hashlib.sha256(raw).hexdigest(), excerpts=excerpts[:160])
        except OSError as error:
            emit("sdk_source_error", path=str(path), error=str(error))
    emit("sdk_inventory_note", files=len(found),
         note="Local SDK sources are not firmware version or proof of active control ownership.")


def ros_check(emit):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from std_msgs.msg import Bool
    from unitree_api.msg import Request, Response

    class Probe(Node):
        def __init__(self):
            super().__init__("g1_loco_readonly_check")
            self.enabled = None
            self.unsafe = False
            self.own_ids = set()
            self.expected = None
            self.answer = None
            self.counts = Counter()
            self.samples = defaultdict(list)
            self.pub = None
            self.create_subscription(
                Bool, "/g1/control_enabled", self.on_enabled,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                           reliability=ReliabilityPolicy.RELIABLE),
            )
            self.create_subscription(Request, "/api/sport/request", self.on_request,
                QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.create_subscription(Response, "/api/sport/response", self.on_response, 100)

        def on_enabled(self, message):
            self.enabled = message.data
            self.unsafe = self.unsafe or message.data

        def on_request(self, message):
            if message.header.identity.id in self.own_ids:
                return
            api_id = message.header.identity.api_id
            self.counts[api_id] += 1
            # Bound log volume and avoid logging unrelated API payloads.
            if api_id in (7105, 7101, 7110, 7111, 1008) and len(self.samples[api_id]) < 5:
                self.samples[api_id].append({
                    "id": message.header.identity.id,
                    "received_monotonic_ns": time.monotonic_ns(),
                    "parameter": message.parameter[:512],
                    "priority": message.header.policy.priority,
                    "lease_id": message.header.lease.id,
                })

        def on_response(self, message):
            identity = (message.header.identity.id, message.header.identity.api_id)
            if identity == self.expected and self.answer is None:
                self.answer = message

        def spin_for(self, seconds):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                if self.unsafe:
                    raise RuntimeError("control_enabled=true observed; stop this check and disarm explicitly")

        def endpoints(self):
            for topic in ("/api/sport/request", "/api/sport/response"):
                emit("publishers", topic=topic, entries=[{
                    "node": endpoint.node_name,
                    "namespace": endpoint.node_namespace,
                    "gid": bytes(endpoint.endpoint_gid).hex(),
                    "qos": str(endpoint.qos_profile),
                } for endpoint in self.get_publishers_info_by_topic(topic)])

        def query(self, api_id):
            request_id = time.monotonic_ns()
            request = read_request(Request, api_id, request_id)
            self.own_ids.add(request_id)
            self.expected = (request_id, api_id)
            self.answer = None
            started = time.monotonic()
            self.pub.publish(request)
            emit("read_query_sent", api=api_id, name=READ_QUERIES[api_id], id=request_id)
            while self.answer is None and time.monotonic() - started < 5.0:
                self.spin_for(0.05)
            self.expected = None
            if self.answer is None:
                emit("read_query_timeout", api=api_id, id=request_id, timeout_s=5)
                return False
            emit("read_query_response", api=api_id, id=request_id,
                 code=self.answer.header.status.code, data=self.answer.data[:2048],
                 elapsed_ms=round((time.monotonic() - started) * 1000, 2))
            return self.answer.header.status.code == 0

    rclpy.init(args=[])
    node = Probe()
    try:
        node.spin_for(3)
        if node.enabled is not False:
            raise RuntimeError("No control_enabled=false received; keep the project bridge running disarmed")
        node.endpoints()
        # The only application-level publisher: three allowlisted getters, no retries.
        node.pub = node.create_publisher(Request, "/api/sport/request", 10)
        node.spin_for(2)
        results = [node.query(api_id) for api_id in READ_QUERIES]
        emit("passive_observation", seconds=OBSERVE_SECONDS,
             instruction="Leave sticks neutral; do not run a motion probe during this check.")
        node.spin_for(OBSERVE_SECONDS)
        node.endpoints()
        emit("other_requests", counts=dict(node.counts), samples=dict(node.samples),
             note="Observed while disarmed. Humble Python does not attribute each message to a GID. "
                  "Traffic does not prove override; absence here does not exclude arbitration during motion.")
        emit("finished", own_read_requests=len(node.own_ids), all_queries_ok=all(results),
             control_enabled=node.enabled)
        return 0 if all(results) else 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def main():
    if len(sys.argv) != 1:
        print("Usage: python3 scripts/hardware_loco_check.py (no API or motion arguments)", file=sys.stderr)
        return 2
    expected = {"ROS_DISTRO": "humble", "ROS_DOMAIN_ID": "0", "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp"}
    if any(os.environ.get(key) != value for key, value in expected.items()):
        print("Source scripts/hardware_env.sh enP8p1s0 on the robot first.", file=sys.stderr)
        return 2
    log_dir = Path(__file__).resolve().parents[1] / "g1_hardware_logs"
    log_dir.mkdir(exist_ok=True)
    path = log_dir / f"loco_check_{datetime.now():%Y%m%d_%H%M%S}_{os.getpid()}.log"
    print(f"Writing locomotion diagnostic log to: {path}", flush=True)
    with path.open("x", encoding="utf-8") as log:
        def emit(event, **values):
            line = json.dumps({"event": event, **values}, ensure_ascii=False)
            print(line, file=log, flush=True)
            print(line, flush=True)
        emit("scope", read_apis=READ_QUERIES, note="No velocity, stop, arming or mode-switch commands. "
             "No SDK imports or firmware changes. Keep the bridge disarmed and controller sticks neutral.")
        try:
            sdk_inventory(emit)
            return ros_check(emit)
        except KeyboardInterrupt:
            emit("interrupted", note="No stop command sent: this diagnostic never commands movement.")
            return 130
        except Exception:
            emit("error", traceback=traceback.format_exc())
            return 2


if __name__ == "__main__":
    raise SystemExit(main())
