from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import Node
import os 
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    config = os.path.join(get_package_share_directory('uuv_compass_driver'), 'config', 'compass_driver_params.yaml')
    
    return LaunchDescription([
        DeclareLaunchArgument('timer_period', default_value='0.1'),
        Node(
        package='uuv_compass_driver',
        executable='compass_driver_node',
        name='compass_driver_node',
        parameters=[config]
        )
    ])