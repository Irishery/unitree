from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = str(
        Path(get_package_share_directory("g1_bridge")) / "config" / "g1_29dof.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            Node(
                package="g1_bridge",
                executable="g1_bridge_node",
                name="g1_bridge",
                output="screen",
                parameters=[LaunchConfiguration("config")],
            ),
        ]
    )

