cmake_minimum_required(VERSION 3.8)
project(uuv_joystick_hal)

# Use ament_cmake for building ROS 2 Python packages with messages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Python3 COMPONENTS Interpreter REQUIRED)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ActionCommand.msg"
  "msg/UUVCommand.msg"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

# Install executable script
install(PROGRAMS
  bin/joystick_hal
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files so `ros2 launch` can find them
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

# Install the Python package sources into the workspace's local dist-packages
execute_process(
  COMMAND ${Python3_EXECUTABLE} -c "import sys; print('python{0}.{1}'.format(sys.version_info.major, sys.version_info.minor))"
  OUTPUT_VARIABLE PYTHON_VERSION_STR
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
install(DIRECTORY
  uuv_joystick_hal
  DESTINATION local/lib/${PYTHON_VERSION_STR}/dist-packages
)
