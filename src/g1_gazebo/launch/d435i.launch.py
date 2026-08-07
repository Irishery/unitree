from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    realsense_share = Path(get_package_share_directory("realsense2_camera"))
    package_share = Path(get_package_share_directory("g1_gazebo"))

    rviz = LaunchConfiguration("rviz")
    serial_no = LaunchConfiguration("serial_no")

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(realsense_share / "launch" / "rs_launch.py")),
        launch_arguments={
            "camera_namespace": "camera",
            "camera_name": "camera",
            "device_type": "d435i",
            "serial_no": serial_no,
            "pointcloud.enable": "true",
            "enable_gyro": "true",
            "enable_accel": "true",
            "unite_imu_method": "2",
        }.items(),
    )

    # realsense2_camera publishes the camera-internal TF tree.  This identity
    # transform attaches that tree to Unitree's factory d435_link mount.
    camera_mount_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "d435_link",
            "--child-frame-id", "camera_link",
        ],
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", str(package_share / "rviz" / "g1.rviz")],
        condition=IfCondition(rviz),
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "serial_no",
                default_value="''",
                description="Optional RealSense serial number; prefix it with an underscore.",
            ),
            DeclareLaunchArgument("rviz", default_value="false"),
            camera,
            camera_mount_tf,
            rviz_node,
        ]
    )
