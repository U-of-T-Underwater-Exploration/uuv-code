cmake_minimum_required(VERSION 3.8)
project(uuv_dvl_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(uuv_msgs REQUIRED)

add_executable(uuv_dvl_driver_node
  src/uuv_dvl_driver.cpp
  src/uuv_dvl_vel_parser.cpp
  src/uuv_dvl_serial_reader.cpp
)

target_include_directories(uuv_dvl_driver_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(uuv_dvl_driver_node
  rclcpp
  geometry_msgs
  sensor_msgs
  diagnostic_msgs
  uuv_msgs
)

install(TARGETS
  uuv_dvl_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
