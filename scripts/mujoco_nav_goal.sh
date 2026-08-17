#!/usr/bin/env bash
set -euo pipefail

show_feedback=false
if [[ "${1:-}" == "--feedback" ]]; then
  show_feedback=true
  shift
fi

if [[ $# -lt 2 || $# -gt 3 ]]; then
  echo "Usage: $0 [--feedback] X Y [YAW_RAD]" >&2
  echo "Example: $0 1.0 0.0" >&2
  echo "Example with full Nav2 feedback: $0 --feedback 1.0 0.0" >&2
  exit 2
fi

goal_x="$1"
goal_y="$2"
goal_yaw="${3:-0.0}"
container_name="${MUJOCO_CONTAINER:-unitree-g1-mujoco}"
feedback_arg=""
if [[ "${show_feedback}" == "true" ]]; then
  feedback_arg="--feedback"
fi

read -r goal_z goal_w < <(
  python3 - "$goal_yaw" <<'PY'
import math
import sys

yaw = float(sys.argv[1])
print(math.sin(yaw * 0.5), math.cos(yaw * 0.5))
PY
)

docker exec "${container_name}" bash -lc "
  source /opt/ros/jazzy/setup.bash
  source /ws/install/setup.bash
  wait_lifecycle() {
    local node=\"\$1\"
    local state=\"\"
    for _ in \$(seq 1 60); do
      state=\$(ros2 lifecycle get \"\${node}\" 2>/dev/null || true)
      if grep -q \"active\" <<<\"\${state}\"; then
        return 0
      fi
      sleep 0.5
    done
    echo \"Timed out waiting for \${node} to become active; last state: \${state}\" >&2
    return 1
  }
  wait_topic_once() {
    local topic=\"\$1\"
    if ! timeout 20 ros2 topic echo \"\${topic}\" --once >/dev/null; then
      echo \"Timed out waiting for first message on \${topic}\" >&2
      return 1
    fi
  }

  echo \"Waiting for Nav2 lifecycle, SLAM map and MuJoCo navigation topics...\"
  wait_lifecycle /controller_server
  wait_lifecycle /planner_server
  wait_topic_once /odom
  wait_topic_once /map
  wait_topic_once /scan
  wait_topic_once /local_costmap/costmap
  wait_topic_once /global_costmap/costmap

  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    \"{pose: {header: {frame_id: map}, pose: {position: {x: ${goal_x}, y: ${goal_y}, z: 0.0}, orientation: {z: ${goal_z}, w: ${goal_w}}}}}\" \
    ${feedback_arg}
"
