"""Legacy safe alias for hardware_telemetry.launch.py.

Use hardware_motion.launch.py with both explicit safety acknowledgements for
the high-level locomotion command interface.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_share = Path(get_package_share_directory("g1_bridge"))
    telemetry_launch = (
        package_share
        / "launch"
        / "hardware_telemetry.launch.py"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config", default_value=str(package_share / "config" / "g1_29dof.yaml")
            ),
            DeclareLaunchArgument(
                "odom_source", default_value="/state_estimator/odom_pelvis"
            ),
            DeclareLaunchArgument("pelvis_height", default_value="0.793"),
            LogInfo(
                msg="bridge.launch.py now starts telemetry only; use the separately "
                "gated hardware_motion.launch.py for physical locomotion"
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(telemetry_launch)),
                launch_arguments={
                    "config": LaunchConfiguration("config"),
                    "odom_source": LaunchConfiguration("odom_source"),
                    "pelvis_height": LaunchConfiguration("pelvis_height"),
                }.items(),
            ),
        ]
    )
