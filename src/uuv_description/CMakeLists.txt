cmake_minimum_required(VERSION 3.22)
project(uuv_description)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install URDF
install(
  DIRECTORY launch urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
