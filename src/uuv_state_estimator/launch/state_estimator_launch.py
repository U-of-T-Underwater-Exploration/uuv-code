from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uuv_state_estimator',      # package of the execuatble
            namespace='utux/',                  # namespace it will be put under
            executable='state_estimator_node',  # exe name (as defined in your setup.py)
            name='state_estimator',             # runtime name of the node
            parameters=[{
                'publish_rate': 50.0,           # --> [ Hz ]      
                'frame_id': 'odom'  
            }]
        )
    ])