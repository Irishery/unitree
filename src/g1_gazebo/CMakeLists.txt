cmake_minimum_required(VERSION 3.8)
project(g1_gazebo)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(d435_pointcloud_optical_relay src/d435_pointcloud_optical_relay.cpp)
ament_target_dependencies(d435_pointcloud_optical_relay rclcpp sensor_msgs)
add_executable(g1_tabletop_pick_demo src/g1_tabletop_pick_demo.cpp)
ament_target_dependencies(
  g1_tabletop_pick_demo
  builtin_interfaces
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  trajectory_msgs
  visualization_msgs)

install(DIRECTORY config launch rviz worlds DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/prepare_urdf.py DESTINATION lib/${PROJECT_NAME})
install(TARGETS d435_pointcloud_optical_relay g1_tabletop_pick_demo DESTINATION lib/${PROJECT_NAME})

ament_package()
