#!/usr/bin/env python3
"""Prepare the official G1 URDF for Gazebo Harmonic without editing upstream."""

from pathlib import Path
import argparse
import xml.etree.ElementTree as ET


CONTROLLED_JOINTS = (
    # Keep the whole articulated model in its nominal posture.  Without these
    # controllers, free, unactuated legs inject collision impulses into the
    # torso while only the right arm is commanded.
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
    "left_hand_thumb_0_joint",
    "left_hand_thumb_1_joint",
    "left_hand_thumb_2_joint",
    "left_hand_middle_0_joint",
    "left_hand_middle_1_joint",
    "left_hand_index_0_joint",
    "left_hand_index_1_joint",
    "right_hand_thumb_0_joint",
    "right_hand_thumb_1_joint",
    "right_hand_thumb_2_joint",
    "right_hand_middle_0_joint",
    "right_hand_middle_1_joint",
    "right_hand_index_0_joint",
    "right_hand_index_1_joint",
)

ARM_POSITION_CONTROLLERS = "\n".join(
    f'''    <plugin filename="gz-sim-joint-position-controller-system"
            name="gz::sim::systems::JointPositionController">
      <joint_name>{joint}</joint_name>
      <topic>/model/g1/joint/{joint}/cmd_pos</topic>
      <p_gain>100.0</p_gain>
      <d_gain>5.0</d_gain>
    </plugin>'''
    for joint in CONTROLLED_JOINTS
)


MODEL_PLUGINS = f"""
  <!-- Added by g1_gazebo for ROS 2 / Gazebo Harmonic. -->
  <gazebo>
    <plugin filename="gz-sim-joint-state-publisher-system"
            name="gz::sim::systems::JointStatePublisher" />
    <plugin filename="gz-sim-velocity-control-system"
            name="gz::sim::systems::VelocityControl">
      <topic>/model/g1/cmd_vel</topic>
    </plugin>
    <plugin filename="gz-sim-pose-publisher-system"
            name="gz::sim::systems::PosePublisher">
      <publish_link_pose>false</publish_link_pose>
      <publish_sensor_pose>false</publish_sensor_pose>
      <publish_collision_pose>false</publish_collision_pose>
      <publish_visual_pose>false</publish_visual_pose>
      <publish_model_pose>true</publish_model_pose>
      <use_pose_vector_msg>true</use_pose_vector_msg>
      <static_publisher>false</static_publisher>
      <update_frequency>50</update_frequency>
      <topic>/model/g1/pose</topic>
    </plugin>
    <plugin filename="gz-sim-odometry-publisher-system"
            name="gz::sim::systems::OdometryPublisher">
      <odom_frame>odom</odom_frame>
      <robot_base_frame>pelvis</robot_base_frame>
      <odom_publish_frequency>50</odom_publish_frequency>
      <odom_topic>/model/g1/odometry</odom_topic>
      <tf_topic>/model/g1/odometry_pose</tf_topic>
      <dimensions>2</dimensions>
    </plugin>
{ARM_POSITION_CONTROLLERS}
    <!-- DEX3 fingers are articulated and commanded in the demo. The
         detachable joint only approximates stable object contact. -->
    <plugin filename="gz-sim-detachable-joint-system"
            name="gz::sim::systems::DetachableJoint">
      <!-- Gazebo collapses the fixed palm joint into the wrist link. -->
      <parent_link>right_wrist_yaw_link</parent_link>
      <child_model>pickup_box</child_model>
      <child_link>link</child_link>
      <attach_topic>/g1/task/grasp/attach</attach_topic>
      <detach_topic>/g1/task/grasp/detach</detach_topic>
    </plugin>
  </gazebo>
  <gazebo reference="torso_link">
    <sensor name="torso_imu" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <topic>/model/g1/imu</topic>
    </sensor>
  </gazebo>
  <gazebo reference="d435_link">
    <!-- Gazebo RGBD approximation of the Intel RealSense D435i.  Topic names
         are bridged to the same ROS namespace as realsense2_camera. -->
    <sensor name="d435i_rgbd" type="rgbd_camera">
      <always_on>true</always_on>
      <visualize>true</visualize>
      <update_rate>30</update_rate>
      <topic>/camera/camera</topic>
      <!-- Gazebo cameras use +X forward.  Publish the raw cloud in this
           body frame; the ROS relay rotates its point coordinates into the
           ROS optical frame before exposing the public point-cloud topic. -->
      <gz_frame_id>d435_link</gz_frame_id>
      <camera>
        <optical_frame_id>d435_link</optical_frame_id>
        <horizontal_fov>1.211259</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
        </image>
        <clip>
          <near>0.10</near>
          <far>10.0</far>
        </clip>
      </camera>
    </sensor>
    <sensor name="d435i_imu" type="imu">
      <always_on>true</always_on>
      <update_rate>200</update_rate>
      <topic>/camera/camera/imu</topic>
      <gz_frame_id>d435_link</gz_frame_id>
    </sensor>
  </gazebo>
  <gazebo reference="mid360_scan">
    <!-- A horizontal scan derived from the stock Livox Mid-360 mounting
         position.  Nav2 and SLAM Toolbox consume this planar representation;
         the physical sensor still requires its real PointCloud2 driver. -->
    <sensor name="mid360_planar" type="gpu_lidar">
      <always_on>true</always_on>
      <visualize>true</visualize>
      <update_rate>15</update_rate>
      <topic>/model/g1/scan</topic>
      <gz_frame_id>mid360_scan</gz_frame_id>
      <lidar>
        <scan>
          <horizontal>
            <samples>900</samples>
            <resolution>1</resolution>
            <min_angle>-3.141592653589793</min_angle>
            <max_angle>3.141592653589793</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.15</min>
          <max>30.0</max>
          <resolution>0.01</resolution>
        </range>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </lidar>
    </sensor>
  </gazebo>
"""


D435_DESCRIPTION = """
  <!-- Optical frames added by g1_gazebo.  The d435_link pose itself comes
       from Unitree's official G1 description. -->
  <link name="d435_color_optical_frame" />
  <joint name="d435_color_optical_joint" type="fixed">
    <origin xyz="0 0 0" rpy="-1.5707963267948966 0 -1.5707963267948966" />
    <parent link="d435_link" />
    <child link="d435_color_optical_frame" />
  </joint>
  <link name="d435_depth_optical_frame" />
  <joint name="d435_depth_optical_joint" type="fixed">
    <origin xyz="0 0 0" rpy="0 0 0" />
    <parent link="d435_color_optical_frame" />
    <child link="d435_depth_optical_frame" />
  </joint>
  <link name="d435_imu_optical_frame" />
  <joint name="d435_imu_optical_joint" type="fixed">
    <origin xyz="0 0 0" rpy="0 0 0" />
    <parent link="d435_color_optical_frame" />
    <child link="d435_imu_optical_frame" />
  </joint>
"""


MID360_SCAN_DESCRIPTION = """
  <!-- The stock Mid-360 frame follows the physical upside-down mounting.
       This child cancels that fixed rotation and provides the level 2D scan
       frame expected by SLAM Toolbox and Nav2. -->
  <link name="mid360_scan" />
  <joint name="mid360_scan_joint" type="fixed">
    <origin xyz="0 0 0" rpy="3.141592653589793 0.05112069379091391 0" />
    <parent link="mid360_link" />
    <child link="mid360_scan" />
  </joint>
"""


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("destination", type=Path)
    args = parser.parse_args()

    source = args.source.resolve()
    text = source.read_text(encoding="utf-8")
    if "</robot>" not in text:
        raise RuntimeError(f"{source} is not a URDF robot file")

    mesh_uri = f"file://{source.parent}/meshes/"
    text = text.replace('filename="meshes/', f'filename="{mesh_uri}')
    text = text.replace(
        '<link name="d435_link"></link>',
        '''<link name="d435_link">
    <visual>
      <origin xyz="0.015 0 0" rpy="0 0 0" />
      <geometry><box size="0.025 0.090 0.025" /></geometry>
      <material name="d435_black"><color rgba="0.06 0.06 0.06 1" /></material>
    </visual>
  </link>''',
    )
    if "<!-- mid360 -->" not in text:
        raise RuntimeError("Could not locate the D435 insertion point in upstream URDF")
    text = text.replace("  <!-- mid360 -->", D435_DESCRIPTION + "\n  <!-- mid360 -->")
    mid360_joint_end = "  </joint>\n\n  <!-- Arm -->"
    if mid360_joint_end not in text:
        raise RuntimeError("Could not locate the Mid-360 joint in upstream URDF")
    text = text.replace(
        mid360_joint_end,
        "  </joint>\n" + MID360_SCAN_DESCRIPTION + "\n  <!-- Arm -->",
        1,
    )
    # The upstream description has inertias and collisions but no Gazebo joint
    # controllers. Disable link gravity for this kinematic GUI / bridge smoke
    # test, otherwise the humanoid immediately collapses under physics.
    robot = ET.fromstring(text)
    link_names = [link.attrib["name"] for link in robot.findall("link")]
    gravity_overrides = "\n".join(
        f'  <gazebo reference="{name}"><gravity>false</gravity></gazebo>'
        for name in link_names
    )
    text = text.replace(
        "</robot>", MODEL_PLUGINS + "\n" + gravity_overrides + "\n</robot>"
    )
    args.destination.parent.mkdir(parents=True, exist_ok=True)
    args.destination.write_text(text, encoding="utf-8")


if __name__ == "__main__":
    main()
