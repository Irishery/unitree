#!/usr/bin/env bash
set -euo pipefail

xhost +si:localuser:root >/dev/null

# Default to software GL: the amdgpu hardware path in Docker currently
# segfaults inside Ogre (GL vertex buffer creation) on this laptop, and
# RViz's Map displays crash the GL context even on llvmpipe (handled by
# the rviz profiles).  Override with RVIZ_GL=auto/hardware if the host
# driver stack works again.
rviz_gl="${RVIZ_GL:-software}"
rviz_profile="${RVIZ_PROFILE:-nav}"
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"
rviz_dir="${repo_root}/src/g1_mujoco/rviz"

case "${rviz_profile}" in
  nav)
    rviz_file="g1_mujoco_nav.rviz"
    ;;
  full)
    rviz_file="g1_mujoco.rviz"
    ;;
  lite)
    rviz_file="g1_mujoco_lite.rviz"
    ;;
  *)
    echo "Unknown RVIZ_PROFILE=${rviz_profile}. Use nav, full or lite." >&2
    exit 2
    ;;
esac

rviz_config="/tmp/unitree_rviz/${rviz_file}"
wait_nav="${RVIZ_WAIT_NAV:-true}"

ros_env_args=()
for name in ROS_DOMAIN_ID RMW_IMPLEMENTATION CYCLONEDDS_URI FASTRTPS_DEFAULT_PROFILES_FILE; do
  if [[ -n "${!name:-}" ]]; then
    ros_env_args+=(-e "${name}=${!name}")
  fi
done

docker_args=(
  --rm
  --network host
  --ipc=host
  "${ros_env_args[@]}"
  -e "DISPLAY=${DISPLAY:-:0}"
  -e QT_X11_NO_MITSHM=1
  # NOTE: no QT_OPENGL=desktop here.  Forcing Qt onto GLX made RViz emit
  # "failed to create drawable" and left the window uncreated on this
  # machine; letting Qt pick its own platform works with software GL.
  -e XDG_RUNTIME_DIR=/tmp/runtime-root
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw
  -v "${rviz_dir}:/tmp/unitree_rviz:ro"
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
  privileged)
    docker_args+=(
      --privileged
      -v /dev:/dev
    )
    ;;
  software)
    # Exact env proven stable on this laptop: force Mesa software
    # rendering without driver overrides (LIBGL_ALWAYS_SOFTWARE picks
    # llvmpipe; MESA_LOADER_DRIVER_OVERRIDE broke GLX drawable creation
    # in a container).
    docker_args+=(
      -e LIBGL_ALWAYS_SOFTWARE=1
      -e MESA_GL_VERSION_OVERRIDE=3.3
      -e MESA_GLSL_VERSION_OVERRIDE=330
    )
    ;;
  llvmpipe)
    docker_args+=(
      -e LIBGL_ALWAYS_SOFTWARE=1
      -e MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
      -e GALLIUM_DRIVER=llvmpipe
      -e MESA_GL_VERSION_OVERRIDE=3.3
      -e MESA_GLSL_VERSION_OVERRIDE=330
    )
    ;;
  indirect)
    docker_args+=(-e LIBGL_ALWAYS_INDIRECT=1)
    ;;
  diagnose)
    docker_args+=(
      --privileged
      -v /dev:/dev
      -e LIBGL_DEBUG=verbose
    )
    docker run "${docker_args[@]}" \
      unitree-g1-mujoco:jazzy \
      bash -lc '
        set -u
        echo "DISPLAY=${DISPLAY:-}"
        echo "QT_OPENGL=${QT_OPENGL:-}"
        echo "LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE:-}"
        echo "MESA_LOADER_DRIVER_OVERRIDE=${MESA_LOADER_DRIVER_OVERRIDE:-}"
        echo
        echo "/tmp/.X11-unix:"
        ls -la /tmp/.X11-unix || true
        echo
        echo "/dev/dri:"
        ls -la /dev/dri || true
        echo
        echo "glxinfo -B:"
        glxinfo -B || true
      '
    exit 0
    ;;
  *)
    echo "Unknown RVIZ_GL=${rviz_gl}. Use auto, hardware, privileged, software, llvmpipe, indirect or diagnose." >&2
    exit 2
    ;;
esac

if [[ "${wait_nav}" == "true" ]]; then
  docker run "${docker_args[@]}" \
    unitree-g1-mujoco:jazzy \
    bash -lc '
      source /opt/ros/jazzy/setup.bash
      source /ws/install/setup.bash
      echo "Waiting for MuJoCo/Nav2 ROS graph before starting RViz..."
      for _ in $(seq 1 80); do
        if ros2 action list 2>/dev/null | grep -qx "/navigate_to_pose" &&
           ros2 topic list 2>/dev/null | grep -qx "/map" &&
           ros2 topic list 2>/dev/null | grep -qx "/global_costmap/costmap"; then
          exec rviz2 -d "'"${rviz_config}"'" "$@"
        fi
        sleep 0.5
      done
      echo "Warning: /navigate_to_pose, /map or /global_costmap/costmap not ready; launching RViz anyway." >&2
      exec rviz2 -d "'"${rviz_config}"'" "$@"
    ' bash "$@"
else
  docker run "${docker_args[@]}" \
    unitree-g1-mujoco:jazzy \
    rviz2 -d "${rviz_config}" "$@"
fi
