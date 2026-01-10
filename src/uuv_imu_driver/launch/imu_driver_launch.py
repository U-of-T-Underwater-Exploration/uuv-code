from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('timer_period', default_value='0.1'),
        Node(
        package='uuv_imu_driver',
        executable='imu_publisher',
        name='imu_driver_node',
        parameters=[{
        'timer_period': LaunchConfiguration('timer_period')
        }]
        )
    ])