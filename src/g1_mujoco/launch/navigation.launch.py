from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
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
            "amcl.ros__parameters.base_frame_id": "base_footprint",
            "bt_navigator.ros__parameters.robot_base_frame": "base_footprint",
            "local_costmap.local_costmap.ros__parameters.robot_base_frame": "base_footprint",
            "global_costmap.global_costmap.ros__parameters.robot_base_frame": "base_footprint",
            "behavior_server.ros__parameters.robot_base_frame": "base_footprint",
            "collision_monitor.ros__parameters.base_frame_id": "base_footprint",
            "docking_server.ros__parameters.base_frame": "base_footprint",
            "local_costmap.local_costmap.ros__parameters.robot_radius": "0.18",
            "global_costmap.global_costmap.ros__parameters.robot_radius": "0.18",
            # The MuJoCo GUI + RViz path can run slower than headless on a
            # laptop.  Keep Nav2 tolerant to scan/TF processing jitter so a
            # short viewer/render hiccup does not abort an otherwise valid
            # goal.
            "amcl.ros__parameters.transform_tolerance": "1.0",
            "bt_navigator.ros__parameters.transform_tolerance": "1.0",
            "local_costmap.local_costmap.ros__parameters.transform_tolerance": "1.0",
            "global_costmap.global_costmap.ros__parameters.transform_tolerance": "1.0",
            "behavior_server.ros__parameters.transform_tolerance": "1.0",
            "collision_monitor.ros__parameters.transform_tolerance": "1.0",
            "collision_monitor.ros__parameters.source_timeout": "3.0",
            "controller_server.ros__parameters.costmap_update_timeout": "2.0",
            "planner_server.ros__parameters.costmap_update_timeout": "2.0",
            "controller_server.ros__parameters.controller_frequency": "20.0",
            "planner_server.ros__parameters.expected_planner_frequency": "5.0",
            "velocity_smoother.ros__parameters.smoothing_frequency": "10.0",
            "velocity_smoother.ros__parameters.odom_duration": "1.0",
            # The default Jazzy bringup uses MPPI with reverse motion enabled
            # and stochastic trajectory sampling. In this kinematic MuJoCo
            # bench that led to oscillations around the tabletop: spin, back
            # up, retry.  Use Regulated Pure Pursuit for deterministic forward
            # path following from point A to point B.
            "controller_server.ros__parameters.failure_tolerance": "1.0",
            "controller_server.ros__parameters.progress_checker.required_movement_radius": "0.15",
            "controller_server.ros__parameters.progress_checker.movement_time_allowance": "20.0",
            "controller_server.ros__parameters.general_goal_checker.xy_goal_tolerance": "0.35",
            "controller_server.ros__parameters.general_goal_checker.yaw_goal_tolerance": "1.57",
            "controller_server.ros__parameters.FollowPath.plugin": "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController",
            "controller_server.ros__parameters.FollowPath.desired_linear_vel": "0.22",
            "controller_server.ros__parameters.FollowPath.lookahead_dist": "0.55",
            "controller_server.ros__parameters.FollowPath.min_lookahead_dist": "0.30",
            "controller_server.ros__parameters.FollowPath.max_lookahead_dist": "0.80",
            "controller_server.ros__parameters.FollowPath.lookahead_time": "1.5",
            "controller_server.ros__parameters.FollowPath.use_velocity_scaled_lookahead_dist": "false",
            "controller_server.ros__parameters.FollowPath.rotate_to_heading_angular_vel": "0.55",
            "controller_server.ros__parameters.FollowPath.use_rotate_to_heading": "true",
            "controller_server.ros__parameters.FollowPath.rotate_to_heading_min_angle": "0.35",
            "controller_server.ros__parameters.FollowPath.max_angular_accel": "1.2",
            "controller_server.ros__parameters.FollowPath.allow_reversing": "false",
            # The synthetic Mid-360 projection feeds both Nav2 costmaps with
            # the correct geometry (the mid360_scan frame is aligned with
            # base_footprint), so RPP's time-to-collision check now sees the
            # same obstacles as the planner.  Keep it enabled as one more
            # safety layer on top of the hard AABB guard in sim.py.
            "controller_server.ros__parameters.FollowPath.use_collision_detection": "true",
            "controller_server.ros__parameters.FollowPath.max_allowed_time_to_collision_up_to_carrot": "1.0",
            "controller_server.ros__parameters.FollowPath.use_regulated_linear_velocity_scaling": "true",
            "controller_server.ros__parameters.FollowPath.use_cost_regulated_linear_velocity_scaling": "true",
            "controller_server.ros__parameters.FollowPath.cost_scaling_dist": "0.45",
            "controller_server.ros__parameters.FollowPath.cost_scaling_gain": "1.0",
            "controller_server.ros__parameters.FollowPath.regulated_linear_scaling_min_radius": "0.55",
            "controller_server.ros__parameters.FollowPath.regulated_linear_scaling_min_speed": "0.08",
            "controller_server.ros__parameters.FollowPath.min_approach_linear_velocity": "0.06",
            "controller_server.ros__parameters.FollowPath.approach_velocity_scaling_dist": "0.60",
            "controller_server.ros__parameters.FollowPath.transform_tolerance": "1.0",
            "controller_server.ros__parameters.FollowPath.stateful": "true",
            "controller_server.ros__parameters.min_y_velocity_threshold": "0.001",
            "local_costmap.local_costmap.ros__parameters.inflation_layer.inflation_radius": "0.45",
            "local_costmap.local_costmap.ros__parameters.inflation_layer.cost_scaling_factor": "5.0",
            "global_costmap.global_costmap.ros__parameters.inflation_layer.inflation_radius": "0.45",
            "global_costmap.global_costmap.ros__parameters.inflation_layer.cost_scaling_factor": "5.0",
            "local_costmap.local_costmap.ros__parameters.width": "5",
            "local_costmap.local_costmap.ros__parameters.height": "5",
            "behavior_server.ros__parameters.max_rotational_vel": "0.75",
            "behavior_server.ros__parameters.min_rotational_vel": "0.15",
            "behavior_server.ros__parameters.rotational_acc_lim": "1.2",
            # Include the high-mounted Mid-360 scan origin in the local voxel
            # layer height window.  Nav2's voxel grid supports at most 16
            # z-values; with the default 0.05 m resolution this spans
            # 0.60..1.40 m relative to base_footprint, covering the simulated
            # scan origin above the lifted pelvis.
            "local_costmap.local_costmap.ros__parameters.voxel_layer.origin_z": "0.6",
            "local_costmap.local_costmap.ros__parameters.voxel_layer.z_voxels": "16",
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

    # RViz2 has no display for nav2_msgs/VoxelGrid; relay the local costmap
    # voxel grid into PointCloud2 so the 3D obstacle grid is visualisable.
    voxel_relay = Node(
        package="g1_mujoco",
        executable="voxel_grid_relay",
        name="voxel_grid_relay",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("slam", default_value="true"),
        slam_toolbox,
        nav2,
        voxel_relay,
    ])
