from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uuv_pwm_driver',
            executable='servo_converter_node',
            name='servo_converter',
            namespace='utux',
            parameters=[
                {
                    'pwm_frequency_hz': 50,
                    'servo_min_pulse_width': 500,
                    'servo_max_pulse_width': 2500,
                    'servo_min_angle': 0,
                    'servo_max_angle': 180,
                }
            ]
        )
    ])
