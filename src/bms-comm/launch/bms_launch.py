from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyS0',
            description='Serial port for BMS communication'
        ),
        DeclareLaunchArgument(
            'baud_rate',
            default_value='115200',
            description='Baud rate for BMS communication'
        ),
        DeclareLaunchArgument(
            'publish_frequency',
            default_value='2.0',
            description='Frequency (Hz) to poll and publish BMS data'
        ),
        Node(
            package='bms-comm',
            executable='bms_node',
            name='bms_node',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'publish_frequency': LaunchConfiguration('publish_frequency'),
            }]
        )
    ])
