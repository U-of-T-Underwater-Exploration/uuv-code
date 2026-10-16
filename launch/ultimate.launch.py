from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():


    motion_converter_launch = os.path.join(
        get_package_share_directory('uuv_motion_converter'),
        'launch',
        'bringup.launch.py'
    )


    joystick_pkg_share = get_package_share_directory('uuv_joystick_hal')

    joystick_hal_launch = os.path.join(
        joystick_pkg_share,
        'launch',
        'joystick_hal.launch.py'
    )

    description_pkg_share = get_package_share_directory('uuv_description')

    description_launch = os.path.join(
        description_pkg_share,
        'launch',
        'uuv_tf.launch.py'
    )

    pwm_launch = os.path.join(
        get_package_share_directory('uuv_pwm_driver'),
        'launch',
        'bringup.launch.py'
    )


    launch_files = [motion_converter_launch, 
                    joystick_hal_launch, 
                    description_launch,
                    pwm_launch]

    launch_descriptions = [IncludeLaunchDescription(launch_file) for launch_file in launch_files]

    return LaunchDescription(launch_descriptions)
