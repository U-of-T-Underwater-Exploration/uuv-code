colcon build; source install/setup.bash
ros2 launch launch/ultimate.launch.py
SDL_JOYSTICK_DEVICE=/dev/input/js0 ros2 run joy joy_node --ros-args -r __ns:=/utux