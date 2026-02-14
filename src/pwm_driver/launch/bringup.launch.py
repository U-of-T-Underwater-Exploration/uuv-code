from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_share = get_package_share_directory('uuv_pwm_driver')

    pwm_driver_launch = os.path.join(
        pkg_share,
        'launch',
        'pwm_driver.launch.py'
    )

    servo_converter_launch = os.path.join(
        pkg_share,
        'launch',
        'servo_converter.launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pwm_driver_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(servo_converter_launch)
        ),
    ])