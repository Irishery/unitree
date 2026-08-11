#!/usr/bin/env bash
set -euo pipefail

if docker ps -a --format '{{.Names}}' | grep -qx unitree-g1-mujoco; then
  docker rm -f unitree-g1-mujoco
fi
