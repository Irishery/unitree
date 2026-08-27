"""Launch the physical head-mounted Livox Mid-360 only.

This launch does not start the locomotion interface, SLAM, Nav2, or any
Unitree command publisher.  It requires the separately installed official
``livox_ros_driver2`` package.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = Path(get_package_share_directory("g1_bridge"))
    default_config = str(
        package_share / "config" / "g1_mid360_192_168_123_164.json"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            DeclareLaunchArgument("frame_id", default_value="mid360_link"),
            DeclareLaunchArgument("publish_freq", default_value="10.0"),
            Node(
                package="livox_ros_driver2",
                executable="livox_ros_driver2_node",
                name="g1_mid360",
                output="screen",
                parameters=[
                    {
                        # 0 is Livox PointCloud2, not the proprietary message.
                        "xfer_format": 0,
                        "multi_topic": 0,
                        "data_src": 0,
                        "publish_freq": ParameterValue(
                            LaunchConfiguration("publish_freq"), value_type=float
                        ),
                        "output_data_type": 0,
                        "frame_id": LaunchConfiguration("frame_id"),
                        "user_config_path": LaunchConfiguration("config"),
                        "cmdline_input_bd_code": "livox0000000001",
                    }
                ],
                remappings=[
                    ("livox/lidar", "/mid360/points"),
                    ("livox/imu", "/mid360/imu"),
                ],
            ),
        ]
    )
