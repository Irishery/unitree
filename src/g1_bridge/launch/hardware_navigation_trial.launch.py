"""First bounded Nav2-to-legs trial for the physical G1.

Requires an already healthy /map, /scan and map->odom transform. The launch
starts disarmed and never publishes low-level motor commands.
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_true(value):
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _launch_navigation_trial(context):
    distro = os.environ.get("ROS_DISTRO", "")
    rmw = os.environ.get("RMW_IMPLEMENTATION", "")
    domain = os.environ.get("ROS_DOMAIN_ID", "")
    if distro != "humble" or rmw != "rmw_cyclonedds_cpp" or domain != "0":
        raise RuntimeError(
            "Physical G1 navigation requires ROS_DISTRO=humble, "
            "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp and ROS_DOMAIN_ID=0. "
            "Source scripts/hardware_env.sh first."
        )
    acknowledgements = (
        LaunchConfiguration("motion_interface").perform(context),
        LaunchConfiguration("allow_hardware_motion").perform(context),
        LaunchConfiguration("allow_nav2_motion").perform(context),
    )
    if not all(_as_true(value) for value in acknowledgements):
        raise RuntimeError(
            "Hardware Nav2 motion is locked. Pass motion_interface:=true, "
            "allow_hardware_motion:=true and allow_nav2_motion:=true only after "
            "the bounded 0.20 m/s bridge test has passed."
        )

    package_share = Path(get_package_share_directory("g1_bridge"))
    params_file = LaunchConfiguration("params_file")
    motion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(package_share / "launch" / "hardware_motion.launch.py")
        ),
        launch_arguments={
            "motion_interface": "true",
            "allow_hardware_motion": "true",
            "cmd_vel_topic": "/g1/motion_cmd_vel",
            "max_linear_x": "0.20",
            "max_linear_y": "0.0",
            "max_angular_z": "0.10",
        }.items(),
    )

    return [
        motion_launch,
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[params_file],
        ),
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[params_file],
            remappings=[("cmd_vel", "/g1/nav_cmd_vel_raw")],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            # Keep Nav2's conventional name: the RViz Navigation 2 panel
            # queries /lifecycle_manager_navigation/is_active.
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[
                {
                    "use_sim_time": False,
                    "autostart": True,
                    "node_names": ["planner_server", "controller_server"],
                }
            ],
        ),
        Node(
            package="g1_bridge",
            executable="nav_velocity_guard.py",
            name="g1_nav_velocity_guard",
            output="screen",
            parameters=[
                {
                    "input_topic": "/g1/nav_cmd_vel_raw",
                    "output_topic": "/g1/motion_cmd_vel",
                    "gait_speed": 0.20,
                    "max_angular_speed": 0.10,
                    "command_timeout": 0.20,
                    "publish_rate": 20.0,
                }
            ],
        ),
        Node(
            package="g1_bridge",
            executable="navigation_goal_bridge.py",
            name="g1_navigation_goal_bridge",
            output="screen",
            parameters=[
                {
                    "min_path_length": 0.10,
                    # Zero disables the former trial-only upper distance bound.
                    "max_path_length": 0.0,
                    # Zero disables the former trial-only accumulated-turn limit.
                    "max_path_heading_change": 0.0,
                }
            ],
        ),
    ]


def generate_launch_description():
    package_share = Path(get_package_share_directory("g1_bridge"))
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=str(
                    package_share / "config" / "hardware_navigation_trial.yaml"
                ),
            ),
            DeclareLaunchArgument("motion_interface", default_value="false"),
            DeclareLaunchArgument("allow_hardware_motion", default_value="false"),
            DeclareLaunchArgument("allow_nav2_motion", default_value="false"),
            OpaqueFunction(function=_launch_navigation_trial),
        ]
    )
