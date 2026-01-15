from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='uuv_motion_converter',
            executable='motion_converter_node',
            name='motion_converter_node',
            parameters=["src/uuv_motion_converter/config/params.yaml"]
        )
    ])