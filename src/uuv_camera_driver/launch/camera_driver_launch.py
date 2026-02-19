from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

# The VM does not have space for gscam
# The pi may need to install, sudo apt install ros-humble-gscam
# To run image, ros2 run image_view image_view image:=/image_raw/compressed
def generate_launch_description():
    return LaunchDescription([

        SetEnvironmentVariable(
            name='GSCAM_CONFIG',
            value=(
                'libcamerasrc ! '
                'video/x-raw,width=640,height=480,framerate=15/1 ! '
                'videoconvert ! '
                'jpegenc quality=70 ! '
                'appsink drop=true sync=false'
            )
        ),

        Node(
            package='gscam',
            executable='gscam_node',
            name='uuv_camera_driver',
            output='screen',
            parameters=[{
                'use_sim_time': False,
            }]
        )
    ])
