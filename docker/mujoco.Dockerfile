FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions python3-pip python3-matplotlib \
    python3-pandas libgl1 libglfw3 assimp-utils \
    ros-jazzy-robot-state-publisher ros-jazzy-rviz2 ros-jazzy-sensor-msgs-py \
    && rm -rf /var/lib/apt/lists/* \
    && pip3 install --break-system-packages --no-cache-dir \
      --index-url https://download.pytorch.org/whl/cpu torch==2.5.1+cpu \
    && pip3 install --break-system-packages --no-cache-dir \
      mujoco==3.3.6 gymnasium==1.0.0 cloudpickle farama-notifications \
    && pip3 install --break-system-packages --no-cache-dir --no-deps \
      stable-baselines3==2.5.0

COPY vendor/g1_description /opt/unitree_ros/robots/g1_description
COPY src/g1_mujoco /ws/src/g1_mujoco
RUN python3 /ws/src/g1_mujoco/scripts/prepare_mjcf.py \
      /opt/unitree_ros/robots/g1_description/g1_29dof_with_hand_rev_1_0.xml \
      /opt/unitree_ros/robots/g1_description/g1_29dof_with_dex3_tabletop.xml \
    && . /opt/ros/jazzy/setup.sh \
    && cd /ws && colcon build --symlink-install --packages-select g1_mujoco \
    && for mesh in /opt/unitree_ros/robots/g1_description/meshes/*.STL; do \
         assimp export "$mesh" "${mesh%.STL}.dae" >/dev/null; \
       done \
    && cp -a /opt/unitree_ros/robots/g1_description/meshes \
      /ws/install/g1_mujoco/share/g1_mujoco/meshes

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "g1_mujoco", "sim.launch.py"]
