"""Passive mapping for the physical G1.

This launch has no Nav2, cmd_vel, Unitree sport API, DEX3, or low-level motor
interface. It projects the physical Mid-360 cloud into a filtered planar scan
and lets SLAM Toolbox publish map -> odom.
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    distro = os.environ.get("ROS_DISTRO", "")
    rmw = os.environ.get("RMW_IMPLEMENTATION", "")
    domain = os.environ.get("ROS_DOMAIN_ID", "")
    if distro != "humble" or rmw != "rmw_cyclonedds_cpp" or domain != "0":
        raise RuntimeError(
            "Physical mapping requires ROS_DISTRO=humble, "
            "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp and explicit ROS_DOMAIN_ID=0. "
            "Source scripts/hardware_env.sh first."
        )

    package_share = Path(get_package_share_directory("g1_bridge"))
    slam_share = Path(get_package_share_directory("slam_toolbox"))
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("slam", default_value="true"),
            DeclareLaunchArgument("cloud_topic", default_value="/mid360/points"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("base_frame", default_value="base_footprint"),
            DeclareLaunchArgument("min_height", default_value="0.12"),
            DeclareLaunchArgument("max_height", default_value="1.60"),
            DeclareLaunchArgument("range_min", default_value="0.55"),
            DeclareLaunchArgument("range_max", default_value="10.0"),
            DeclareLaunchArgument("cloud_frame_stride", default_value="2"),
            Node(
                package="g1_bridge",
                executable="mid360_scan_projector",
                name="g1_mid360_scan_projector",
                output="screen",
                parameters=[
                    {
                        "cloud_topic": LaunchConfiguration("cloud_topic"),
                        "scan_topic": LaunchConfiguration("scan_topic"),
                        "base_frame": LaunchConfiguration("base_frame"),
                        "min_height": ParameterValue(
                            LaunchConfiguration("min_height"), value_type=float
                        ),
                        "max_height": ParameterValue(
                            LaunchConfiguration("max_height"), value_type=float
                        ),
                        "range_min": ParameterValue(
                            LaunchConfiguration("range_min"), value_type=float
                        ),
                        "range_max": ParameterValue(
                            LaunchConfiguration("range_max"), value_type=float
                        ),
                        "frame_stride": ParameterValue(
                            LaunchConfiguration("cloud_frame_stride"), value_type=int
                        ),
                    }
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(slam_share / "launch" / "online_async_launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "autostart": "true",
                    "slam_params_file": str(package_share / "config" / "hardware_slam_toolbox.yaml"),
                }.items(),
                condition=IfCondition(LaunchConfiguration("slam")),
            ),
        ]
    )
