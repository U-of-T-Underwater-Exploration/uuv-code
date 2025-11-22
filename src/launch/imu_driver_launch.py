from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    imu_driver_node = Node(
        package='uuv_imu_driver',
        executable='imu_publisher',
        name='imu_driver_node',
        parameters=[{                   # define constants for use inside node
                'publish_rate': 50.0,   # --> [ Hz ]       
                'frame_id': 'imu_link'  
            }])

    return LaunchDescription([
        imu_driver_node
    ])