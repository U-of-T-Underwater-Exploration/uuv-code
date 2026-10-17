cmake_minimum_required(VERSION 3.8)
project(uuv_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SafetyStatus.msg"
  "msg/KoggerDvlVel.msg"
  DEPENDENCIES std_msgs
)

ament_package()