#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pwm_msg.msg import ServoPWM
from pwm_msg.msg import CommandPWM

class ServoConverter(Node):

    def __init__(self):
        super().__init__('servo_converter')
        self.servo_command_sub_ = self.create_subscription(ServoPWM, '/pwm/servo', self.servo_callback, 10)
        self.command_pub = self.create_publisher(CommandPWM, '/pwm/command', 10)

        self.declare_parameter('servo_min_pulse_width', 500)  #the pulse width, in microseconds, for 0 deg angle for the servo
        self.declare_parameter('servo_max_pulse_width', 2500)  #the pulse width, in microseconds, for 180 deg angle for the servo 

        self.declare_parameter('servo_min_angle', 0)  #Minimum angle for the servo (in deg)
        self.declare_parameter('servo_max_angle', 180)  #Maximum angle for the servo (in deg)

        self.declare_parameter('pwm_frequency_hz', 50)  #PWM frequency

    def angle_to_duty_cycle(self, angle):
        """
        Convert servo angle to duty cycle
        """
        servo_max_pulse_width = self.get_parameter('servo_max_pulse_width').value
        servo_min_pulse_width = self.get_parameter('servo_min_pulse_width').value

        angle_max = self.get_parameter('servo_max_angle').value
        angle_min = self.get_parameter('servo_min_angle').value

        if angle > angle_max:
            self.get_logger().warn('Given angle above maximum angle, clamping')
            angle = angle_max
        
        elif angle < angle_min:
            self.get_logger().warn('Given angle below minimum angle. Clamping')
            angle = angle_min

        angle_range = (angle_max - angle_min)  
        pulse_width_range = (servo_max_pulse_width - servo_min_pulse_width)  

        if angle_range == 0:
            self.get_logger().error('Invalid angle range, division by zero: Setting duty cycle to zero')
            return 0.0
        
        pulse_width = (((angle - angle_min) * pulse_width_range) / angle_range) + servo_min_pulse_width

        freq = self.get_parameter('pwm_frequency_hz').value

        return float(pulse_width * freq / 1000000)

    def servo_callback(self, msg):
        """
        Publish PWM duty cycle for a servo along with its corresponding channel
        """

        servo_angle = msg.servo_angle   
        channel = msg.pwm_channelnumber

        dty_cycle = self.angle_to_duty_cycle(servo_angle)

        cmd_msg = CommandPWM()
        cmd_msg.pwm_values = dty_cycle
        cmd_msg.pwm_channelnumber = channel

        self.command_pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    servo_converter_node = ServoConverter()
    rclpy.spin(servo_converter_node)
    servo_converter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()