#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
docker build -f "${workspace_dir}/docker/mujoco.Dockerfile" -t unitree-g1-mujoco:jazzy "${workspace_dir}"
