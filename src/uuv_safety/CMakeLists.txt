cmake_minimum_required(VERSION 3.8)
project(uuv_safety)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(safety_monitor src/safety_monitor.cpp)
ament_target_dependencies(safety_monitor rclcpp sensor_msgs)
target_compile_features(safety_monitor PUBLIC cxx_std_17)

install(TARGETS safety_monitor
  DESTINATION lib/${PROJECT_NAME})

ament_package()