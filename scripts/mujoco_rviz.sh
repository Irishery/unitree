#!/usr/bin/env bash
set -euo pipefail

xhost +si:localuser:root >/dev/null

rviz_gl="${RVIZ_GL:-auto}"

docker_args=(
  --rm
  --network host
  --ipc=host
  -e "DISPLAY=${DISPLAY:-:0}"
  -e QT_X11_NO_MITSHM=1
  -e QT_OPENGL=desktop
  -e XDG_RUNTIME_DIR=/tmp/runtime-root
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw
)

case "${rviz_gl}" in
  auto)
    if [[ -e /dev/dri ]]; then
      docker_args+=(--device /dev/dri:/dev/dri)
    else
      docker_args+=(
        -e LIBGL_ALWAYS_SOFTWARE=1
        -e MESA_GL_VERSION_OVERRIDE=3.3
        -e MESA_GLSL_VERSION_OVERRIDE=330
      )
    fi
    ;;
  hardware)
    docker_args+=(--device /dev/dri:/dev/dri)
    ;;
  software)
    docker_args+=(
      -e LIBGL_ALWAYS_SOFTWARE=1
      -e MESA_GL_VERSION_OVERRIDE=3.3
      -e MESA_GLSL_VERSION_OVERRIDE=330
    )
    ;;
  indirect)
    docker_args+=(-e LIBGL_ALWAYS_INDIRECT=1)
    ;;
  *)
    echo "Unknown RVIZ_GL=${rviz_gl}. Use auto, hardware, software or indirect." >&2
    exit 2
    ;;
esac

docker run "${docker_args[@]}" \
  unitree-g1-mujoco:jazzy \
  rviz2 -d /ws/install/g1_mujoco/share/g1_mujoco/rviz/g1_mujoco.rviz "$@"
