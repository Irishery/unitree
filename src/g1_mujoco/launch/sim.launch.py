import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("g1_mujoco"))
    description_dir = Path(os.environ.get("G1_DESCRIPTION_DIR", "/opt/unitree_ros/robots/g1_description"))
    robot_description = (description_dir / "g1_29dof_with_hand_rev_1_0.urdf").read_text(encoding="utf-8")
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
        DeclareLaunchArgument("rviz", default_value="true"),
        Node(
            package="g1_mujoco",
            executable="sim",
            name="g1_mujoco",
            output="screen",
            parameters=[{"viewer": LaunchConfiguration("viewer")}],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            remappings=[("joint_states", "/g1/joint_states")],
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
        Node(
            package="g1_mujoco",
            executable="box_detector",
            name="g1_box_detector",
            output="screen",
        ),
        Node(
            package="g1_mujoco",
            executable="pick_controller",
            name="g1_pick_controller",
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(package_share / "rviz" / "g1_mujoco.rviz")],
            condition=IfCondition(LaunchConfiguration("rviz")),
            output="screen",
        ),
    ])
