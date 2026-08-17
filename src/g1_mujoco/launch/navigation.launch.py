from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    package_share = Path(get_package_share_directory("g1_mujoco"))
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
            "robot_radius": "0.18",
            # The MuJoCo GUI + RViz path can run slower than headless on a
            # laptop.  Keep Nav2 tolerant to scan/TF processing jitter so a
            # short viewer/render hiccup does not abort an otherwise valid
            # goal.
            "transform_tolerance": "1.0",
            "source_timeout": "3.0",
            "costmap_update_timeout": "2.0",
            "controller_frequency": "20.0",
            "expected_planner_frequency": "5.0",
            "smoothing_frequency": "10.0",
            "odom_duration": "1.0",
            # Include the high-mounted Mid-360 scan origin in the local voxel
            # layer height window.  Nav2's voxel grid supports at most 16
            # z-values; with the default 0.05 m resolution this spans
            # -0.20..0.60 m relative to the pelvis frame, covering the
            # simulated scan origin at z ~= 0.47 m.
            "origin_z": "-0.2",
            "z_voxels": "16",
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

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("slam", default_value="true"),
        slam_toolbox,
        nav2,
    ])
