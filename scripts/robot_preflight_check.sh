#!/usr/bin/env bash
set -u

# Read-only Unitree G1 ROS 2 preflight.
# This script never publishes commands and never enables robot control.

SCRIPT_NAME="$(basename "$0")"
RUN_ID="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${G1_PREFLIGHT_OUT_DIR:-./g1_preflight_logs}"
LOG="${OUT_DIR}/preflight_${RUN_ID}.log"

mkdir -p "$OUT_DIR"

log_cmd() {
  echo
  echo "\$ $*"
  "$@"
}

log_shell() {
  echo
  echo "\$ $*"
  bash -lc "$*"
}

section() {
  echo
  echo "===== $* ====="
}

topic_section() {
  echo
  echo "----- $* -----"
}

unique_lines() {
  awk '!seen[$0]++'
}

candidate_setups() {
  {
    printf '%s\n' \
      /home/unitree/unitree_ros2/install/setup.bash \
      /home/unitree/g1_dev/unitree_ros2/example/install/setup.bash \
      /home/unitree/g1_bottle_ws/install/setup.bash \
      /home/unitree/g1_grasp_ws/install/setup.bash \
      /home/unitree/dex3_ws/install/setup.bash \
      /home/unitree/unitree_ros2/cyclonedds_ws/install/setup.bash

    find /home/unitree -path '*/install/setup.bash' 2>/dev/null || true
  } | unique_lines
}

setup_has_unitree_msgs() {
  local setup_file="$1"
  bash -lc "
    set +u
    source /opt/ros/humble/setup.bash
    source '${setup_file}'
    set -u
    ros2 pkg prefix unitree_hg >/dev/null
    ros2 pkg prefix unitree_api >/dev/null
    ros2 pkg prefix unitree_go >/dev/null
  " >/dev/null 2>&1
}

select_unitree_setup() {
  local setup_file
  while IFS= read -r setup_file; do
    [ -f "$setup_file" ] || continue
    if setup_has_unitree_msgs "$setup_file"; then
      printf '%s\n' "$setup_file"
      return 0
    fi
  done < <(candidate_setups)
  return 1
}

run_topic_info() {
  local topic="$1"
  topic_section "$topic"
  ros2 topic info -v "$topic" || true
}

run_topic_hz() {
  local topic="$1"
  topic_section "$topic"
  timeout 8 ros2 topic hz "$topic" || true
}

run_topic_echo_once() {
  local topic="$1"
  topic_section "$topic"
  timeout 5 ros2 topic echo "$topic" --once || true
}

run_checks() {
  section "SAFETY"
  echo "Read-only preflight for Unitree G1."
  echo "This script does NOT publish to /cmd_vel, /api/*/request, /lowcmd, /user_lowcmd, hand cmd topics, or enable services."
  echo "Keep the physical remote/controller available. Do not run motion tests from this script."

  section "SYSTEM"
  log_cmd hostname || true
  log_cmd uname -a || true
  log_cmd lsb_release -a || true
  log_shell "command -v ros2" || true
  echo
  echo "ROS_DISTRO=${ROS_DISTRO:-}"
  echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-}"
  echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-}"
  log_shell "printenv | grep -E 'ROS_|RMW|CYCLONEDDS|FASTRTPS|UNITREE' | sort" || true
  log_shell "ros2 doctor --report 2>&1 | sed -n '1,120p'" || true

  section "UNITREE SETUP DISCOVERY"
  echo "Candidate setup.bash files:"
  candidate_setups || true
  echo
  if [ -n "${SELECTED_UNITREE_SETUP:-}" ]; then
    echo "Selected setup: ${SELECTED_UNITREE_SETUP}"
  else
    echo "Selected setup: <none>"
    echo "WARNING: unitree_hg/unitree_api/unitree_go were not found together."
    echo "         Native Unitree topics may be visible but ros2 echo/hz can fail to deserialize them."
  fi

  section "UNITREE PACKAGE CHECK"
  log_shell "ros2 pkg prefix unitree_hg" || true
  log_shell "ros2 pkg prefix unitree_api" || true
  log_shell "ros2 pkg prefix unitree_go" || true
  log_shell "ros2 interface show unitree_hg/msg/LowState | head -80" || true
  log_shell "ros2 interface show unitree_hg/msg/SportModeState | head -80" || true
  log_shell "ros2 interface show unitree_go/msg/SportModeState | head -80" || true
  log_shell "ros2 interface show unitree_api/msg/Request | head -80" || true
  log_shell "ros2 interface show unitree_go/msg/WirelessController | head -80" || true

  section "TOPIC LIST"
  log_shell "ros2 topic list | sort" || true

  section "RELEVANT TOPICS"
  log_shell "ros2 topic list | grep -Ei 'low|high|state|sport|api|imu|odom|lidar|livox|point|cloud|scan|hand|dex|tf|wireless|emergency|slam' | sort" || true

  section "TF PRESENCE"
  log_shell "ros2 topic list | grep -E '^/tf$|^/tf_static$'" || true
  run_topic_info /tf
  run_topic_info /tf_static

  section "TOPIC INFO"
  local info_topics=(
    /lowstate
    /lf/lowstate
    /lowstate_doubleimu
    /sportmodestate
    /lf/sportmodestate
    /api/sport/request
    /api/sport/response
    /state_estimator/odom_pelvis
    /state_estimator/odom_torso
    /state_estimator/fusion_odom
    /state_estimator/fusion_odom_torso
    /unitree_slam/high_rate_odometry
    /unitree/slam_mapping/odom
    /unitree/slam_mapping/points
    /unitree/slam_relocation/odom
    /unitree/slam_relocation/points
    /utlidar/range_info
    /slam_info
    /slam_key_info
    /wirelesscontroller
    /lf/emergency_stop
    /secondary_imu
    /lf/secondary_imu
    /dex3/left/state
    /dex3/right/state
    /lf/dex3/left/state
    /lf/dex3/right/state
  )
  local topic
  for topic in "${info_topics[@]}"; do
    run_topic_info "$topic"
  done

  section "TOPIC HZ"
  local hz_topics=(
    /lowstate
    /lf/lowstate
    /lowstate_doubleimu
    /sportmodestate
    /lf/sportmodestate
    /state_estimator/odom_pelvis
    /state_estimator/odom_torso
    /state_estimator/fusion_odom
    /state_estimator/fusion_odom_torso
    /unitree_slam/high_rate_odometry
    /unitree/slam_mapping/odom
    /unitree/slam_mapping/points
    /unitree/slam_relocation/odom
    /unitree/slam_relocation/points
    /utlidar/range_info
    /wirelesscontroller
    /lf/emergency_stop
  )
  for topic in "${hz_topics[@]}"; do
    run_topic_hz "$topic"
  done

  section "ONE MESSAGE SAMPLES"
  local sample_topics=(
    /lowstate
    /lf/lowstate
    /sportmodestate
    /state_estimator/odom_pelvis
    /state_estimator/odom_torso
    /state_estimator/fusion_odom
    /unitree_slam/high_rate_odometry
    /unitree/slam_mapping/odom
    /unitree/slam_mapping/points
    /unitree/slam_relocation/odom
    /unitree/slam_relocation/points
    /utlidar/range_info
    /slam_info
    /slam_key_info
    /wirelesscontroller
    /lf/emergency_stop
  )
  for topic in "${sample_topics[@]}"; do
    run_topic_echo_once "$topic"
  done

  section "SUMMARY HINTS"
  echo "Check these in the log:"
  echo "- unitree_hg/unitree_api/unitree_go package prefixes must exist for native echo/hz."
  echo "- /lowstate or /lf/lowstate should echo and have a stable rate."
  echo "- /state_estimator/odom_pelvis should publish nav_msgs/Odometry around 50 Hz."
  echo "- /tf and /tf_static may be absent; if so, our hardware launch must publish TF."
  echo "- /unitree/slam_* points with Publisher count 0 means native SLAM/point cloud is not active yet."
  echo "- Do not test movement until telemetry, odom, TF, lidar/points, remote, and watchdog are verified."
}

{
  echo "Writing preflight log to: ${LOG}"

  # Source ROS first. Unitree setup is selected dynamically below.
  if [ -f /opt/ros/humble/setup.bash ]; then
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
    set -u
  else
    echo "ERROR: /opt/ros/humble/setup.bash not found"
  fi

  SELECTED_UNITREE_SETUP="$(select_unitree_setup || true)"
  export SELECTED_UNITREE_SETUP
  if [ -n "${SELECTED_UNITREE_SETUP}" ]; then
    set +u
    # shellcheck disable=SC1090
    source "${SELECTED_UNITREE_SETUP}"
    set -u
  fi

  run_checks
} 2>&1 | tee "$LOG"

echo
echo "Saved to: ${LOG}"
echo "Send this file back for analysis."
