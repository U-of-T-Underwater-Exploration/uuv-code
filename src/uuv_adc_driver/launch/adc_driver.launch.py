from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uuv_adc_driver',    # package of the execuatble
            namespace='utux/sensor',        # namespace it will be put under
            executable='adc_publisher',     # exe name (as defined in your setup.py)
            name='adc_publisher',           # runtime name of the node
            parameters=[{                   # define constants for use inside node
                'publish_rate': 50.0,   # --> [ Hz ]    
            }]
        )
    ])