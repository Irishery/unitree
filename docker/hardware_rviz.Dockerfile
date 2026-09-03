FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    rm -f /etc/apt/apt.conf.d/docker-clean \
    && apt-get -o Acquire::ForceIPv4=true -o Acquire::Retries=5 \
       -o Acquire::http::Timeout=30 update \
    && apt-get -o Acquire::ForceIPv4=true -o Acquire::Retries=5 \
       -o Acquire::http::Timeout=30 install -y --no-install-recommends \
       ros-humble-rmw-cyclonedds-cpp

ENV ROS_DISTRO=humble
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

CMD ["rviz2"]
