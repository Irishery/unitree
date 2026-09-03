"""Telemetry-only physical G1 launch.

The telemetry executable does not link unitree_api and contains no /cmd_vel,
service, or Unitree sport-request command path.
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _require_hardware_environment() -> None:
    distro = os.environ.get("ROS_DISTRO", "")
    rmw = os.environ.get("RMW_IMPLEMENTATION", "")
    domain = os.environ.get("ROS_DOMAIN_ID", "")
    if distro != "humble":
        raise RuntimeError(
            f"Physical G1 telemetry requires ROS_DISTRO=humble (got {distro!r}). "
            "Source scripts/hardware_env.sh first."
        )
    if rmw != "rmw_cyclonedds_cpp":
        raise RuntimeError(
            "Physical G1 telemetry requires RMW_IMPLEMENTATION=rmw_cyclonedds_cpp "
            f"(got {rmw!r}). Source scripts/hardware_env.sh first."
        )
    if domain != "0":
        raise RuntimeError(
            f"Physical G1 telemetry requires explicit ROS_DOMAIN_ID=0 (got {domain!r})."
        )


def generate_launch_description():
    _require_hardware_environment()

    package_share = Path(get_package_share_directory("g1_bridge"))
    default_config = str(package_share / "config" / "g1_29dof.yaml")
    default_urdf = package_share / "urdf" / "g1_29dof_rev_1_0.urdf"
    robot_description = default_urdf.read_text(encoding="utf-8")
    robot_description = robot_description.replace(
        'filename="meshes/', 'filename="package://g1_bridge/meshes/'
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            DeclareLaunchArgument(
                "odom_source", default_value="/state_estimator/odom_pelvis"
            ),
            DeclareLaunchArgument("pelvis_height", default_value="0.793"),
            Node(
                package="g1_bridge",
                executable="g1_hardware_telemetry_node",
                name="g1_bridge",
                output="screen",
                parameters=[LaunchConfiguration("config")],
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
    )
