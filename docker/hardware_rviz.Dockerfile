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

# The physical robot publishes a URDF whose visual geometry uses
# package://g1_bridge/meshes/... URLs.  RViz runs on the laptop, so it needs a
# local ament package resource and a copy of those meshes to resolve the URLs.
RUN mkdir -p \
      /opt/g1_viewer/share/ament_index/resource_index/packages \
      /opt/g1_viewer/share/g1_bridge \
    && touch /opt/g1_viewer/share/ament_index/resource_index/packages/g1_bridge
COPY src/g1_bridge/package.xml /opt/g1_viewer/share/g1_bridge/package.xml
COPY vendor/g1_description/meshes /opt/g1_viewer/share/g1_bridge/meshes

ENV ROS_DISTRO=humble
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV AMENT_PREFIX_PATH=/opt/g1_viewer:/opt/ros/humble

CMD ["rviz2"]
