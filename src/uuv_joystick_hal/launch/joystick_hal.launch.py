from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'joy',
            executable = 'joy_node',
            name = 'joy_node',
            namespace = 'utux',
        ),
        Node(
            package='uuv_joystick_hal',
            executable='joystick_hal',
            name='joystick_hal',
            namespace = 'utux',
        )
    ])