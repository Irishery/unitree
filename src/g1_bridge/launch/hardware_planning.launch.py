"""Planning-only Nav2 stage for the physical G1.

This launch starts a global costmap and planner_server only. It contains no
controller, behavior tree, velocity smoother, cmd_vel publisher, or Unitree
motion bridge, so a planned path cannot move the robot.
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    distro = os.environ.get("ROS_DISTRO", "")
    rmw = os.environ.get("RMW_IMPLEMENTATION", "")
    domain = os.environ.get("ROS_DOMAIN_ID", "")
    if distro != "humble" or rmw != "rmw_cyclonedds_cpp" or domain != "0":
        raise RuntimeError(
            "Physical G1 planning requires ROS_DISTRO=humble, "
            "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp and ROS_DOMAIN_ID=0. "
            "Source scripts/hardware_env.sh first."
        )

    package_share = Path(get_package_share_directory("g1_bridge"))
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=str(package_share / "config" / "hardware_planner.yaml"),
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[params_file],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_planning",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": False,
                        "autostart": True,
                        "node_names": ["planner_server"],
                    }
                ],
            ),
            Node(
                package="g1_bridge",
                executable="planner_goal_bridge.py",
                name="g1_planner_goal_bridge",
                output="screen",
            ),
        ]
    )
