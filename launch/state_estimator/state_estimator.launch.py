from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('uuv_description'), 'launch', 'uuv_tf.launch.py')
        )
    )

    state_estimator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('uuv_state_estimator'), 'launch', 'state_estimator.launch.py')
        )
    )


    return LaunchDescription([
        tf_launch,
        state_estimator_launch
    ])