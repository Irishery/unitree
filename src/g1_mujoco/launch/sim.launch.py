import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("g1_mujoco"))
    description_dir = Path(os.environ.get("G1_DESCRIPTION_DIR", "/opt/unitree_ros/robots/g1_description"))
    robot_description = (description_dir / "g1_29dof_with_hand_rev_1_0.urdf").read_text(encoding="utf-8")
    navigation = LaunchConfiguration("navigation")
    publish_camera = LaunchConfiguration("publish_camera")
    slam = LaunchConfiguration("slam")
    tabletop_pick = LaunchConfiguration("tabletop_pick")
    loco_api = LaunchConfiguration("loco_api")
    loco_api_bridge = LaunchConfiguration("loco_api_bridge")
    sim_cmd_vel_topic = LaunchConfiguration("sim_cmd_vel_topic")
    sim_smoothed_cmd_vel_topic = LaunchConfiguration("sim_smoothed_cmd_vel_topic")
    # RViz resolves package:// resources through the ament index more reliably
    # than a bind-mounted file URI.  The image build installs these meshes in
    # g1_mujoco's share directory.
    robot_description = robot_description.replace(
        'filename="meshes/', 'filename="package://g1_mujoco/meshes/'
    )
    # RViz/OGRE has robust support for Collada, unlike some binary STL files
    # from the vendor bundle.  Docker converts the installed meshes to DAE.
    robot_description = robot_description.replace(".STL", ".dae")
    return LaunchDescription([
        DeclareLaunchArgument("viewer", default_value="true"),
        DeclareLaunchArgument("viewer_lite", default_value="false"),
        DeclareLaunchArgument("walk", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("publish_camera", default_value="true"),
        DeclareLaunchArgument("navigation", default_value="true"),
        DeclareLaunchArgument("slam", default_value="true"),
        DeclareLaunchArgument("tabletop_pick", default_value="false"),
        DeclareLaunchArgument("loco_api", default_value="false"),
        DeclareLaunchArgument(
            "loco_api_request_topic", default_value="/g1/sim/api/sport/request"),
        DeclareLaunchArgument(
            "loco_api_response_topic", default_value="/g1/sim/api/sport/response"),
        DeclareLaunchArgument("loco_api_cmd_vel_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("loco_api_require_start", default_value="false"),
        DeclareLaunchArgument("loco_api_bridge", default_value="false"),
        DeclareLaunchArgument("loco_api_bridge_cmd_vel_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument(
            "loco_api_bridge_request_topic", default_value="/g1/sim/api/sport/request"),
        DeclareLaunchArgument("sim_cmd_vel_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("sim_smoothed_cmd_vel_topic", default_value="/cmd_vel_smoothed"),
        Node(
            package="g1_mujoco",
            executable="sim",
            name="g1_mujoco",
            output="screen",
            parameters=[{
                "viewer": LaunchConfiguration("viewer"),
                "viewer_lite": LaunchConfiguration("viewer_lite"),
                "publish_camera": publish_camera,
                "tabletop_pick": tabletop_pick,
                "walk": LaunchConfiguration("walk"),
                "cmd_vel_topic": sim_cmd_vel_topic,
                "smoothed_cmd_vel_topic": sim_smoothed_cmd_vel_topic,
            }],
        ),
        Node(
            package="g1_mujoco",
            executable="loco_api_sim",
            name="g1_loco_api_sim",
            condition=IfCondition(loco_api),
            parameters=[{
                "request_topic": LaunchConfiguration("loco_api_request_topic"),
                "response_topic": LaunchConfiguration("loco_api_response_topic"),
                "cmd_vel_topic": LaunchConfiguration("loco_api_cmd_vel_topic"),
                "require_start": LaunchConfiguration("loco_api_require_start"),
            }],
            output="screen",
        ),
        Node(
            package="g1_mujoco",
            executable="cmd_vel_loco_bridge",
            name="g1_cmd_vel_loco_bridge",
            condition=IfCondition(loco_api_bridge),
            parameters=[{
                "cmd_vel_topic": LaunchConfiguration("loco_api_bridge_cmd_vel_topic"),
                "request_topic": LaunchConfiguration("loco_api_bridge_request_topic"),
            }],
            output="screen",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            remappings=[("joint_states", "/g1/joint_states")],
            output="screen",
        ),
        # The latched TRANSIENT_LOCAL /robot_description sample is not
        # delivered to late subscribers in this Fast DDS/Docker stack, so
        # RViz never receives the URDF and renders primitive shapes only.
        # Republish the same text as a volatile topic that the RViz
        # profiles subscribe to (see description_relay.py).
        Node(
            package="g1_mujoco",
            executable="description_relay",
            name="description_relay",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
        # MuJoCo's D435i renderer is fixed in the head recess.  This makes its
        # optical frame available to RViz and consumers of the RGB-D topics.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "0.075", "0", "0.420", "-0.642788", "0.642788", "-0.298836", "0.298836",
                "torso_link", "d435_color_optical_frame",
            ],
            output="screen",
        ),
        # The MuJoCo Mid-360 scan is mounted in the head/torso, like the real
        # robot.  sim.py publishes a real 3D PointCloud2 from this frame and a
        # 2D LaserScan projection for SLAM/Nav2, so the local voxel layer sees
        # a high sensor origin instead of a base-footprint shortcut.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "0.0002835", "0.00003", "0.428434", "0", "0", "0",
                "torso_link", "mid360_scan",
            ],
            output="screen",
        ),
        Node(
            package="g1_mujoco",
            executable="box_detector",
            name="g1_box_detector",
            condition=IfCondition(tabletop_pick),
            output="screen",
        ),
        Node(
            package="g1_mujoco",
            executable="pick_controller",
            name="g1_pick_controller",
            condition=IfCondition(tabletop_pick),
            output="screen",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(package_share / "launch" / "navigation.launch.py")),
            launch_arguments={"use_sim_time": "false", "autostart": "true", "slam": slam}.items(),
            condition=IfCondition(navigation),
        ),
        TimerAction(
            # Give SLAM Toolbox/Nav2 a few seconds to create /map, costmaps and
            # /navigate_to_pose before the Nav2 RViz panel starts querying the
            # action server.  The lightweight nav profile also avoids the RGB
            # Image display that triggers GLSL sampler errors on some Mesa
            # drivers.
            period=12.0,
            actions=[
                Node(
                    package="rviz2",
                    executable="rviz2",
                    arguments=["-d", str(package_share / "rviz" / "g1_mujoco_nav.rviz")],
                    condition=IfCondition(LaunchConfiguration("rviz")),
                    output="screen",
                )
            ],
        ),
    ])
