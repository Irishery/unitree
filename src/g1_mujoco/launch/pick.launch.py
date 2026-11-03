from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    sim = Path(get_package_share_directory("g1_mujoco")) / "launch" / "sim.launch.py"
    return LaunchDescription([
        DeclareLaunchArgument("viewer", default_value="true"),
        DeclareLaunchArgument("bottle_x", default_value="0.42"),
        DeclareLaunchArgument("bottle_y", default_value="0.0"),
        DeclareLaunchArgument("bottle_yaw", default_value="0.0"),
        DeclareLaunchArgument("bottle_diameter", default_value="0.062"),
        DeclareLaunchArgument("bottle_height", default_value="0.235"),
        DeclareLaunchArgument("grasp_side", default_value="right"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(sim)),
            launch_arguments={
                "viewer": LaunchConfiguration("viewer"),
                "tabletop_pick": "true",
                "publish_camera": "true",
                "navigation": "false",
                "slam": "false",
                "walk": "false",
                "bottle_x": LaunchConfiguration("bottle_x"),
                "bottle_y": LaunchConfiguration("bottle_y"),
                "bottle_yaw": LaunchConfiguration("bottle_yaw"),
                "bottle_diameter": LaunchConfiguration("bottle_diameter"),
                "bottle_height": LaunchConfiguration("bottle_height"),
                "grasp_side": LaunchConfiguration("grasp_side"),
            }.items(),
        ),
    ])
