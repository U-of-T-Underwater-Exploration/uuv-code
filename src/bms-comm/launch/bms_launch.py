"""
bms_launch.py
Launch file for the bms_node.

All three parameters can be overridden on the command line, e.g.

    ros2 launch bms-comm bms_launch.py serial_port:=/dev/ttyAMA1 publish_frequency:=5.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([

        # ---- Configurable arguments ----------------------------------------
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyS0",
            description=(
                "UART port connected to the BMS 'GPS' header. "
                "Supported ports on the BlueRobotics Navigator: "
                "Serial1=/dev/ttyS0  Serial3=/dev/ttyAMA1  Serial5=/dev/ttyAMA3"
            ),
        ),

        DeclareLaunchArgument(
            "baud_rate",
            default_value="115200",
            description="Baud rate for BMS UART communication (115200 preferred, 9600 backup).",
        ),

        DeclareLaunchArgument(
            "publish_frequency",
            default_value="2.0",
            description="Rate [Hz] at which the BMS is polled and data published.",
        ),

        # ---- Node ----------------------------------------------------------
        Node(
            package="bms-comm",
            executable="bms_node",
            name="bms_node",
            output="screen",
            parameters=[{
                "serial_port":       LaunchConfiguration("serial_port"),
                "baud_rate":         LaunchConfiguration("baud_rate"),
                "publish_frequency": LaunchConfiguration("publish_frequency"),
            }],
        ),
    ])
