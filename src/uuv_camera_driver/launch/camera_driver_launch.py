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
            namespace='camera', # for /camera
            parameters=[{
                'camera_name': 'uuv_camera',
                'width': 640,
                'height': 480,
                #'FrameDurationLimits': "[50000, 50000]", #camera_ros default is 20Hz
                'format': "RGB888"
            }]
        )
    ])
