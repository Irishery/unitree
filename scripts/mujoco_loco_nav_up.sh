#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Hardware-like command path for MuJoCo:
#   Nav2 /cmd_vel
#     -> cmd_vel_loco_bridge
#     -> /api/sport/request
#     -> loco_api_sim
#     -> /g1/sim/cmd_vel
#     -> sim.py
#
# Extra launch args are appended, so callers can still override viewer_lite,
# walk, publish_camera, etc.
exec "${script_dir}/mujoco_up.sh" \
  navigation:=true \
  slam:=true \
  loco_api:=true \
  loco_api_bridge:=true \
  loco_api_cmd_vel_topic:=/g1/sim/cmd_vel \
  loco_api_bridge_cmd_vel_topic:=/cmd_vel \
  sim_cmd_vel_topic:=/g1/sim/cmd_vel \
  sim_smoothed_cmd_vel_topic:=/g1/sim/cmd_vel_smoothed \
  "$@"
