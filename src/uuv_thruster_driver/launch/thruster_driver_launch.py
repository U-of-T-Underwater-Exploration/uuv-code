from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uuv_thruster_driver',
            executable='thruster_driver_node',
            name='thruster_driver',
        ),
    ])