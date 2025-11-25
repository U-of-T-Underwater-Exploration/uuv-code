cmake_minimum_required(VERSION 3.22)
project(uuv_state_estimator)

## Use C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD)
endif()

## Compile Flags
add_compile_options(-Wall -Wextra -Wpedantic)

## Find ROS2 Packages
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)

## Include directories
include_directories(
  include
  "${CMAKE_SOURCE_DIR}/../extern/eigen"
)

## Add executables
add_executable(state_estimator_node src/state_estimator_node.cpp)
ament_target_dependencies(state_estimator_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2_eigen
)

## Use external dependencies
target_include_directories(state_estimator_node PUBLIC
  "${CMAKE_SOURCE_DIR}/../extern/eigen"
)

## Install targets
install(TARGETS
  state_estimator_node
  DESTINATION lib/${PROJECT_NAME})

## Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
