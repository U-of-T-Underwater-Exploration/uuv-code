from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_share = get_package_share_directory('uuv_motion_converter')

    motion_converter_launch = os.path.join(
        pkg_share,
        'launch',
        'motion_converter.launch.py'
    )

    thruster_driver_launch = os.path.join(
        pkg_share,
        'launch',
        'thruster_driver.launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(motion_converter_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(thruster_driver_launch)
        ),
    ])
