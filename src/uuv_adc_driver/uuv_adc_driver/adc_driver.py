import rclpy
from rclpy.node import Node
import bluerobotics_navigator as navigator

from std_msgs.msg import Float32MultiArray

from math import atan, sqrt, pi

class ADCPublisher(Node):
    def __init__(self):
        super().__init__('adc_publisher')

        # State
        self.adc = float(0)

        # Get Parameters
        self.declare_parameter("publish_rate", 50)
        self.declare_parameter("frame_id", 'fc_link')
        self.timer_period = 1/self.get_parameter("publish_rate").get_parameter_value().double_value

        # Create Publisher & timer
        self.publisher_ = self.create_publisher(Float32MultiArray, 'adc/data', 10)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Initialize Navigator
        navigator.init()
        adcVals = navigator.read_adc_all()

    def timer_callback(self):
        message = Float32MultiArray()

        message.data = self.adcVals

        self.publisher_.publish(message)

def main(args=None):
    rclpy.init(args=args)

    adc_publisher = ADCPublisher()

    rclpy.spin(adc_publisher)

    adc_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()