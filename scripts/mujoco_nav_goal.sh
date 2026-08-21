#!/usr/bin/env bash
set -euo pipefail

show_feedback=false
args=()
for arg in "$@"; do
  case "${arg}" in
    --feedback)
      show_feedback=true
      ;;
    *)
      args+=("${arg}")
      ;;
  esac
done
set -- "${args[@]}"

if [[ $# -lt 2 || $# -gt 3 ]]; then
  echo "Usage: $0 [--feedback] X Y [YAW_RAD]" >&2
  echo "Example: $0 1.20 0.90" >&2
  echo "Example with full Nav2 feedback: $0 --feedback 1.20 0.90" >&2
  echo "Also accepted: $0 1.20 0.90 --feedback" >&2
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

python3 - "$goal_x" "$goal_y" <<'PY'
import sys

x = float(sys.argv[1])
y = float(sys.argv[2])

table_center_x = 0.95
table_half_length = 0.18
table_half_width = 0.35
base_radius = 0.26
margin = 0.05
inflate = base_radius + margin

min_x = table_center_x - table_half_length - inflate
max_x = table_center_x + table_half_length + inflate
min_y = -table_half_width - inflate
max_y = table_half_width + inflate

if min_x <= x <= max_x and min_y <= y <= max_y:
    print(
        "Goal is inside the inflated nav table obstacle "
        f"([{min_x:.2f}, {max_x:.2f}] x [{min_y:.2f}, {max_y:.2f}]).",
        file=sys.stderr,
    )
    print("Use a goal beside/behind the table, for example: 1.20 0.90 or 1.20 -0.90", file=sys.stderr)
    sys.exit(2)
PY

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

  # The first goal after startup can hit a Fast DDS discovery race inside
  # bt_navigator (\"Timed out while waiting for action server to
  # acknowledge goal request for follow_path\") - the request reaches
  # controller_server but the client gives up on the ack and aborts.
  # A quick single resend is enough once the action graph is warm.
  send_goal() {
    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
      \"{pose: {header: {frame_id: map}, pose: {position: {x: ${goal_x}, y: ${goal_y}, z: 0.0}, orientation: {z: ${goal_z}, w: ${goal_w}}}}}\" \
      ${feedback_arg}
  }
  out_file=\$(mktemp)
  if ! send_goal 2>&1 | tee \"\${out_file}\"; then
    echo \"Goal send failed; retrying once...\" >&2
    sleep 3
    send_goal
    rc=\$?
    rm -f \"\${out_file}\"
    exit \${rc}
  fi
  if grep -q \"Goal finished with status: ABORTED\" \"\${out_file}\" && ! grep -q \"number_of_recoveries: [1-9]\" \"\${out_file}\"; then
    echo \"Fast abort (likely first-goal DDS discovery race); retrying once...\" >&2
    sleep 3
    send_goal
    rc=\$?
    rm -f \"\${out_file}\"
    exit \${rc}
  fi
  rm -f \"\${out_file}\"
"
