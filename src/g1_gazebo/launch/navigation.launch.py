from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    package_share = Path(get_package_share_directory("g1_gazebo"))
    nav2_share = Path(get_package_share_directory("nav2_bringup"))
    slam_share = Path(get_package_share_directory("slam_toolbox"))

    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    slam = LaunchConfiguration("slam")

    configured_nav2_params = RewrittenYaml(
        source_file=str(nav2_share / "params" / "nav2_params.yaml"),
        param_rewrites={
            "use_sim_time": use_sim_time,
            "robot_base_frame": "pelvis",
            "base_frame_id": "pelvis",
            "base_frame": "pelvis",
            "robot_radius": "0.32",
        },
        convert_types=True,
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(slam_share / "launch" / "online_async_launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "slam_params_file": str(package_share / "config" / "slam_toolbox.yaml"),
        }.items(),
        condition=IfCondition(slam),
    )

    # Give Gazebo, the bridge and SLAM a moment to establish scan and TF before
    # the Nav2 lifecycle manager activates the costmaps.
    nav2 = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(nav2_share / "launch" / "navigation_launch.py")),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "params_file": configured_nav2_params,
                    "use_composition": "False",
                }.items(),
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("slam", default_value="true"),
            slam_toolbox,
            nav2,
        ]
    )
