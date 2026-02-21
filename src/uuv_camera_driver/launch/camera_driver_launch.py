from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_ros',
            executable='camera_node',
            name='uuv_camera_driver',
            output='screen',
            parameters=[{
                'camera_name': 'uuv_camera',
                'image_width': 640,
                'image_height': 480,
                'framerate': 15.0,
                'device_id': 0,  # typically /dev/video0
                'use_system_default_qos': True
            }]
        )
    ])
