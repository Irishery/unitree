#!/usr/bin/env bash
set -euo pipefail

action="${1:-start}"
container="${MUJOCO_CONTAINER:-unitree-g1-mujoco}"

case "${action}" in
  start)
    topic="/g1/pick/start"
    payload="{data: true}"
    ;;
  cancel|stop)
    topic="/g1/pick/cancel"
    payload="{data: true}"
    ;;
  status)
    docker exec "${container}" bash -lc \
      'source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && timeout 5 ros2 topic echo --once /g1/pick/status'
    exit $?
    ;;
  *)
    echo "Usage: $0 [start|cancel|status]" >&2
    exit 2
    ;;
esac

docker exec "${container}" bash -lc \
  "source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && ros2 topic pub --once ${topic} std_msgs/msg/Bool '${payload}'"
