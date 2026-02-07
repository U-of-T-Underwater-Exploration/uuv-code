# uuv-code
 UTUX Onboard UUV code

Sourcing:
source src/install/setup.bash

colcon build --package-select uuv_baro_ext

Launching:
ros2 launch uuv_baro_ext external_barometer_launch.py