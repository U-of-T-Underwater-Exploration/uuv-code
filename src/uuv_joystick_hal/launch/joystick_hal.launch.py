from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uuv_joystick_hal',
            executable='joystick_hal',
            name='joystick_hal',
            namespace = 'utux',
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            namespace='utux',
            # SDL_JOYSTICK_DEVICE pins joy_node to a specific device path so it
            # doesn't grab the wrong /dev/input/jsX if multiple are present.
            additional_env={'SDL_JOYSTICK_DEVICE': '/dev/input/js0'},
        ),
    ])
