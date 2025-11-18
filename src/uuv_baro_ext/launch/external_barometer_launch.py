import launch
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='uuv_baro_ext',
            namespace='utux/sensor',
            executable='uuv_baro_ext_publisher',
            name='external_barometer_publisher',
            parameters=[{
                'publish_rate': 50.0,
                'frame_id': 'baro_external_link'
                }]
            )
    ])