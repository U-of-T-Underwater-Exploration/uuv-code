from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uuv_pwm_driver',
            executable='pwm_driver_node',
            name='pwm_driver',
            namespace='utux',
            parameters=[
                {
                    'pwm_frequency_hz': 50,
                }
            ]
        )
    ])
