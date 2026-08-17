import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("g1_gazebo"))
    ros_gz_share = Path(get_package_share_directory("ros_gz_sim"))
    description_dir = Path(os.environ.get("G1_DESCRIPTION_DIR", "/opt/unitree_ros/robots/g1_description"))
    urdf_path = description_dir / "g1_29dof_with_dex3_gazebo.urdf"
    world_path = package_share / "worlds" / "g1_world.sdf"
    bridge_path = package_share / "config" / "bridge.yaml"

    headless = LaunchConfiguration("headless")
    rviz = LaunchConfiguration("rviz")
    pick_auto_start = LaunchConfiguration("pick_auto_start")
    navigation = LaunchConfiguration("navigation")
    slam = LaunchConfiguration("slam")
    robot_description = urdf_path.read_text(encoding="utf-8")

    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(ros_gz_share / "launch" / "gz_sim.launch.py")),
        launch_arguments={"gz_args": f"-r -s -v 3 {world_path}", "on_exit_shutdown": "true"}.items(),
        condition=IfCondition(headless),
    )

    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(ros_gz_share / "launch" / "gz_sim.launch.py")),
        launch_arguments={"gz_args": f"-r -v 3 {world_path}", "on_exit_shutdown": "true"}.items(),
        condition=UnlessCondition(headless),
    )

    spawn = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=["-world", "g1_world", "-name", "g1", "-file", str(urdf_path), "-z", "0.80"],
                output="screen",
            )
        ],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="g1_gz_bridge",
        parameters=[{"config_file": str(bridge_path), "use_sim_time": True}],
        output="screen",
    )

    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        remappings=[("joint_states", "/g1/joint_states")],
        output="screen",
    )

    # Gazebo RGBD points use the camera body convention (+X forward), while
    # ROS optical frames use +Z forward.  Rotate point coordinates before
    # exposing the RealSense-compatible public topic.
    pointcloud_relay = Node(
        package="g1_gazebo",
        executable="d435_pointcloud_optical_relay",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    tabletop_pick_demo = Node(
        package="g1_gazebo",
        executable="g1_tabletop_pick_demo",
        parameters=[{"use_sim_time": True, "auto_start": pick_auto_start}],
        output="screen",
    )

    navigation_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(package_share / "launch" / "navigation.launch.py")),
        launch_arguments={"use_sim_time": "true", "autostart": "true", "slam": slam}.items(),
        condition=IfCondition(navigation),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", str(package_share / "rviz" / "g1.rviz")],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(rviz),
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("pick_auto_start", default_value="true"),
            DeclareLaunchArgument("navigation", default_value="true"),
            DeclareLaunchArgument("slam", default_value="true"),
            gazebo_headless,
            gazebo_gui,
            spawn,
            bridge,
            state_publisher,
            pointcloud_relay,
            tabletop_pick_demo,
            navigation_stack,
            rviz_node,
        ]
    )
