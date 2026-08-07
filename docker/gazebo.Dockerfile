FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    rm -f /etc/apt/apt.conf.d/docker-clean \
    && apt-get -o Acquire::ForceIPv4=true -o Acquire::Retries=5 -o Acquire::http::Timeout=30 update \
    && apt-get -o Acquire::ForceIPv4=true -o Acquire::Retries=5 -o Acquire::http::Timeout=30 install -y --no-install-recommends \
    git \
    python3-colcon-common-extensions \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-rviz2 \
    ros-jazzy-teleop-twist-keyboard

COPY vendor/g1_description /opt/unitree_ros/robots/g1_description

WORKDIR /ws
COPY src/g1_gazebo /ws/src/g1_gazebo
RUN python3 /ws/src/g1_gazebo/scripts/prepare_urdf.py \
      /opt/unitree_ros/robots/g1_description/g1_29dof_rev_1_0.urdf \
      /opt/unitree_ros/robots/g1_description/g1_29dof_gazebo.urdf \
    && . /opt/ros/jazzy/setup.sh \
    && colcon build --symlink-install --packages-select g1_gazebo

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "g1_gazebo", "sim.launch.py"]
