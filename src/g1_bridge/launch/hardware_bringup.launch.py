from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = Path(get_package_share_directory("g1_bridge"))
    default_config = str(package_share / "config" / "g1_29dof.yaml")
    default_urdf = package_share / "urdf" / "g1_29dof_rev_1_0.urdf"
    robot_description = default_urdf.read_text(encoding="utf-8")
    robot_description = robot_description.replace(
        'filename="meshes/', 'filename="package://g1_bridge/meshes/'
    )

    motion_interface = LaunchConfiguration("motion_interface")
    dex3 = LaunchConfiguration("dex3")

    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            DeclareLaunchArgument(
                "motion_interface",
                default_value="false",
                description="Expose cmd_vel and Unitree sport API command path",
            ),
            DeclareLaunchArgument(
                "dex3",
                default_value="false",
                description="Start the physical DEX3 bridge (still disarmed by default)",
            ),
            DeclareLaunchArgument(
                "odom_source",
                default_value="/state_estimator/odom_pelvis",
            ),
            DeclareLaunchArgument("pelvis_height", default_value="0.793"),
            Node(
                package="g1_bridge",
                executable="g1_bridge_node",
                name="g1_bridge",
                output="screen",
                parameters=[
                    LaunchConfiguration("config"),
                    {
                        "low_state_topic": "/lowstate",
                        "motion_interface_enabled": ParameterValue(
                            motion_interface, value_type=bool
                        ),
                        "start_control_enabled": False,
                    },
                ],
            ),
            Node(
                package="g1_bridge",
                executable="odom_tf_node",
                name="g1_odom_tf",
                output="screen",
                parameters=[
                    {
                        "input_odom_topic": LaunchConfiguration("odom_source"),
                        "output_odom_topic": "/odom",
                        "odom_frame": "odom",
                        "base_frame": "base_footprint",
                        "pelvis_frame": "pelvis",
                        "pelvis_height": ParameterValue(
                            LaunchConfiguration("pelvis_height"), value_type=float
                        ),
                        "use_input_pelvis_z": False,
                        "publish_tf": True,
                    }
                ],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="g1_robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
                remappings=[("joint_states", "/g1/joint_states")],
            ),
            Node(
                package="g1_bridge",
                executable="dex3_bridge_node",
                name="dex3_bridge",
                output="screen",
                condition=IfCondition(dex3),
                parameters=[LaunchConfiguration("config")],
            ),
        ]
    )
