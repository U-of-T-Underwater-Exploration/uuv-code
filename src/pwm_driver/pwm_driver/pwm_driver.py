#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pwm_msg.msg import ThrustersPWM
from std_msgs.msg import Float32MultiArray
import bluerobotics_navigator as navigator

class PWMDriver(Node):

    def __init__(self):
        super().__init__('pwm_driver')
        
        # Declare parameters
        self.declare_parameter('pwm_frequency_hz', 50)
        
        self.get_logger().info('PWM Driver Node has been started.')
        self.thrusters_sub = self.create_subscription(Float32MultiArray, '/pwm/thrusters', self.thrusters_callback, 10)
        self.pwm_pub = self.create_publisher(ThrustersPWM, '/pwm/generated', 10)

        self.init_navigator()
        self.pwm_values = [0.0] * 16  # Initialize with zero PWM values
        self.pwm_valid = [False] * 16  # Initialize all channels as invalid

    def thrusters_callback(self, msg):
        thruster_duty = msg.data
        if len(thruster_duty) != 8:
            self.get_logger().error('Received PWM values length is not 8.')
            return

        for i, duty in enumerate(thruster_duty):
            self.pwm_valid[i] = True  # Assume valid unless proven otherwise

            if not (0.0 <= duty <= 1.0):
                self.get_logger().error('Thruster PWM values must be between 0.0 and 1.0.')
                duty = max(0.0, min(1.0, duty))
                self.pwm_valid[i] = False

            self.pwm_values[i] = duty

        try:
            self.set_pwm_thruster_channels(self.pwm_values)
        except Exception as e:
            self.get_logger().error(f'Bluenavigator error: {e}')

        self.publish_pwm()

    def set_pwm_thruster_channels(self, values):
        """Set PWM values for channels one to eight."""
        channels = [1, 2, 3, 4, 5, 6, 7, 8]
        values = [v for v in values]
        
        navigator.set_pwm_channels_duty_cycle_values(channels, values)

    def publish_pwm(self):
        """Publish the current PWM values and their validity."""

        pwm_values = self.pwm_values
        pwm_valid = self.pwm_valid

        pwm_msg = ThrustersPWM()
        self.get_logger().info(f'Set PWM values: {pwm_values}')
        self.get_logger().info(f'PWM validity: {pwm_valid}')
        pwm_msg.pwm_values = pwm_values
        pwm_msg.pwm_valid = pwm_valid
        self.pwm_pub.publish(pwm_msg)
    

    def init_navigator(self):
        """Initialize the Blue Robotics Navigator for PWM control.
        : Deploy when hardware is available.
        """
        navigator.init()
        freq = self.get_parameter('pwm_frequency_hz').value
        navigator.set_pwm_freq_hz(freq)
        navigator.set_pwm_enabled(True)


def main(args=None):
    rclpy.init(args=args)
    pwm_driver_node = PWMDriver()
    rclpy.spin(pwm_driver_node)
    pwm_driver_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()