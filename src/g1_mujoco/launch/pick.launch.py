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
        DeclareLaunchArgument("box_x", default_value="0.40"),
        DeclareLaunchArgument("box_y", default_value="0.0"),
        DeclareLaunchArgument("box_yaw", default_value="0.0"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(sim)),
            launch_arguments={
                "viewer": LaunchConfiguration("viewer"),
                "tabletop_pick": "true",
                "publish_camera": "true",
                "navigation": "false",
                "slam": "false",
                "box_x": LaunchConfiguration("box_x"),
                "box_y": LaunchConfiguration("box_y"),
                "box_yaw": LaunchConfiguration("box_yaw"),
            }.items(),
        ),
    ])
