import rclpy
from rclpy.node import Node
try:
    import bluerobotics_navigator as navigator
except Exception:
    navigator = None
import yaml
import os
import time 
import math

from sensor_msgs.msg import MagneticField
from builtin_interfaces.msg import Time


class CompassPublisher(Node):

    def __init__(self):
        super().__init__('compass_publisher')
        self.rawDataPublisher_ = self.create_publisher(MagneticField, 'compass/data_raw', 10)
        self.dataPublisher = self.create_publisher(MagneticField, 'compass/data', 10)
        self.declare_parameter('timer_period', 0.5) # Seconds, should be decided by yaml
        self.declare_parameter('cutoff_frequency', 2.0) # Hz

        self.time_ = time.time()
        
        self.timer_period = self.get_parameter('timer_period').get_parameter_value().double_value
        self.get_logger().info('Timer period set to: %.3f seconds' % self.timer_period)


        try: navigator.init()
        except Exception as err:
            self.get_logger().error(str(err))

        #Filter parameters

        self.cutoff_frequency = self.get_parameter('cutoff_frequency').get_parameter_value().double_value
        self.get_logger().info('Cutoff frequency set to: %.3f seconds' % self.cutoff_frequency)
        self.sample_frequency = 1.0 / self.timer_period
        self.get_logger().info('Sample frequency set to: %.3f seconds' % self.sample_frequency)

        self.b0 = 0.0
        self.b1 = 0.0
        self.a1 = 0.0
        self.calculate_filter_coefficients()
        self.get_logger().info('Filter coefficients calculated: b0=%.3f, b1=%.3f, a1=%.3f' % (self.b0, self.b1, self.a1))

        self.prev_raw_data = MagneticField()
        self.prev_data = MagneticField()

        self.prev_raw_data = self.set_default_values(self.prev_raw_data)
        self.prev_data = self.set_default_values(self.prev_data)
        self.get_logger().info('Previous data initialized to zero values.')

        self.timer = self.create_timer(self.timer_period, self.timer_callback)   


    def timer_callback(self):
        raw_data = MagneticField()
        data = MagneticField()

        magfield = None

        elapsed = time.time() - self.time_
        sec = int(elapsed)
        nanosec = int((elapsed - sec) * 1e9)

        if nanosec < 0:
            nanosec = 0
    

        try: 
            magfield = navigator.read_mag()
        except Exception as err:
            self.get_logger().error(str(err))

        if magfield is not None:
            raw_data = self.set_data(raw_data,magfield)

            data = self.low_pass_filter(raw_data)

            self.get_logger().info('Publishing raw data: MagField[%.3f, %.3f, %.3f]' %
                                   (magfield.x, magfield.y, magfield.z))
        else:
            raw_data = self.set_default_values(raw_data)
            data = self.set_default_values(data)

            self.get_logger().info('Publishing default values: MagField[0, 0, 0]')
            

        raw_data = self.set_header(raw_data, 'compass_link', sec, nanosec)
        data = self.set_header(data, 'compass_link', sec, nanosec)
        
        self.rawDataPublisher_.publish(raw_data)
        self.dataPublisher.publish(data)
        

    def low_pass_filter(self, raw_data):
        data = MagneticField()
        data.magnetic_field.x = self.low_pass_filter_single_axis(raw_data.magnetic_field.x,
                                                                     self.prev_raw_data.magnetic_field.x,
                                                                     self.prev_data.magnetic_field.x)
        data.magnetic_field.y = self.low_pass_filter_single_axis(raw_data.magnetic_field.y,
                                                                     self.prev_raw_data.magnetic_field.y,
                                                                     self.prev_data.magnetic_field.y)
        data.magnetic_field.z = self.low_pass_filter_single_axis(raw_data.magnetic_field.z,
                                                                     self.prev_raw_data.magnetic_field.z,
                                                                     self.prev_data.magnetic_field.z)
        self.prev_raw_data = raw_data
        self.prev_data = data
        
        return data
    
    def low_pass_filter_single_axis(self, raw_value, prev_raw_value, prev_value):
        value = self.b0 * raw_value + self.b1 * prev_raw_value - self.a1 * prev_value
        return value
    
    def set_header(self, data, frame_id, sec, nanosec):
        data.header.frame_id = frame_id
        data.header.stamp.sec = sec
        data.header.stamp.nanosec = nanosec
        return data
    
    def set_default_values(self, data):
        data.magnetic_field.x = 0.0
        data.magnetic_field.y = 0.0
        data.magnetic_field.z = 0.0
        return data
    
    def set_data(self, data, magfield):
        data.magnetic_field.x = magfield.x
        data.magnetic_field.y = magfield.y
        data.magnetic_field.z = magfield.z
        return data

    def calculate_filter_coefficients(self):
        K = math.tan(math.pi * self.cutoff_frequency / self.sample_frequency)
        self.b0 = K / (1 + K)
        self.b1 = self.b0
        self.a1 = (K - 1) / (1 + K)

def main(args=None):
    rclpy.init(args=args)

    compass_driver = CompassPublisher()

    rclpy.spin(compass_driver)

    compass_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
