#!/usr/bin/env bash
set -euo pipefail

docker stop unitree-g1-gazebo >/dev/null 2>&1 || true
xhost -si:localuser:root >/dev/null 2>&1 || true
