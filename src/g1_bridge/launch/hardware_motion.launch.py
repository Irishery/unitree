"""Explicitly gated high-level locomotion bridge for a physical G1.

This launch never starts armed. It only exposes the /cmd_vel command path
after both launch-time safety acknowledgements are set to true.
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _as_true(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _launch_motion(context):
    distro = os.environ.get("ROS_DISTRO", "")
    rmw = os.environ.get("RMW_IMPLEMENTATION", "")
    domain = os.environ.get("ROS_DOMAIN_ID", "")
    if distro != "humble" or rmw != "rmw_cyclonedds_cpp" or domain != "0":
        raise RuntimeError(
            "Physical G1 motion requires ROS_DISTRO=humble, "
            "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp and explicit ROS_DOMAIN_ID=0. "
            "Source scripts/hardware_env.sh first."
        )

    motion_interface = LaunchConfiguration("motion_interface").perform(context)
    acknowledgement = LaunchConfiguration("allow_hardware_motion").perform(context)
    if not (_as_true(motion_interface) and _as_true(acknowledgement)):
        raise RuntimeError(
            "Hardware motion is locked. Pass both motion_interface:=true and "
            "allow_hardware_motion:=true after completing the physical safety checklist."
        )

    package_share = Path(get_package_share_directory("g1_bridge"))
    default_urdf = package_share / "urdf" / "g1_29dof_rev_1_0.urdf"
    robot_description = default_urdf.read_text(encoding="utf-8")
    robot_description = robot_description.replace(
        'filename="meshes/', 'filename="package://g1_bridge/meshes/'
    )

    return [
        Node(
            package="g1_bridge",
            executable="g1_bridge_node",
            name="g1_bridge",
            output="screen",
            parameters=[
                LaunchConfiguration("config"),
                {
                    "low_state_topic": "/lowstate",
                    "motion_interface_enabled": True,
                    "start_control_enabled": False,
                },
            ],
        ),
        Node(
            package="g1_bridge",
            executable="odom_tf_node",
            name="g1_odom_tf",
            output="screen",
            parameters=[
                {
                    "input_odom_topic": LaunchConfiguration("odom_source"),
                    "output_odom_topic": "/odom",
                    "odom_frame": "odom",
                    "base_frame": "base_footprint",
                    "pelvis_frame": "pelvis",
                    "pelvis_height": ParameterValue(
                        LaunchConfiguration("pelvis_height"), value_type=float
                    ),
                    "use_input_pelvis_z": False,
                    "publish_tf": True,
                }
            ],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="g1_robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
            remappings=[("joint_states", "/g1/joint_states")],
        ),
    ]


def generate_launch_description():
    package_share = Path(get_package_share_directory("g1_bridge"))
    default_config = str(package_share / "config" / "g1_29dof.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            DeclareLaunchArgument("motion_interface", default_value="false"),
            DeclareLaunchArgument("allow_hardware_motion", default_value="false"),
            DeclareLaunchArgument(
                "odom_source", default_value="/state_estimator/odom_pelvis"
            ),
            DeclareLaunchArgument("pelvis_height", default_value="0.793"),
            OpaqueFunction(function=_launch_motion),
        ]
    )
