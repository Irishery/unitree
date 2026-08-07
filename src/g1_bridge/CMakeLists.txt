cmake_minimum_required(VERSION 3.8)
project(g1_bridge)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(unitree_api REQUIRED)
find_package(unitree_hg REQUIRED)

add_executable(g1_bridge_node src/g1_bridge_node.cpp)
target_compile_features(g1_bridge_node PUBLIC cxx_std_17)
ament_target_dependencies(g1_bridge_node
  diagnostic_msgs
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  unitree_api
  unitree_hg
)

install(TARGETS g1_bridge_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()

