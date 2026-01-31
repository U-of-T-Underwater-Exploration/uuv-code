cmake_minimum_required(VERSION 3.8)
project(uuv_joystick_hal)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
#add_executable(joystick_hal_node src/uuv_joystick_hal/input_node.py)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ActionCommand.msg"
  "msg/UUVCommand.msg"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
#ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  joystick_hal/input_node.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
