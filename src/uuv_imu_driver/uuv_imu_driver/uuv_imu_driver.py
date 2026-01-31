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

from sensor_msgs.msg import Imu
from builtin_interfaces.msg import Time


class ImuPublisher(Node):

    def __init__(self):
        super().__init__('imu_publisher')
        self.rawDataPublisher_ = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.dataPublisher = self.create_publisher(Imu, 'imu/data', 10)
        self.declare_parameter('timer_period', 0.5)  # seconds
        self.declare_parameter('cutoff_frequency', 2.0)  # Hz
        self.declare_parameter('coordinate_system_linear', [1,1,1])
        self.declare_parameter('coordinate_system_angular', [1,1,1])        

        self.time_ = time.time()
        
        timer_period = self.get_parameter('timer_period').get_parameter_value().double_value
        self.get_logger().info('Timer period set to: %.3f seconds' % timer_period)

        try: navigator.init()
        except Exception as err:
            self.get_logger().error(str(err))
            
        #Filter parameters
        self.coordinate_system_linear = list(self.get_parameter('coordinate_system_linear').value) #might be an array
        self.coordinate_system_angular = list(self.get_parameter('coordinate_system_angular').value) #might be an array
        self.cutoff_frequency = self.get_parameter('cutoff_frequency').get_parameter_value().double_value
        self.get_logger().info('Cutoff frequency set to: %.3f Hz' % self.cutoff_frequency)
        self.sample_frequency = 1.0 / timer_period
        self.get_logger().info('Sample frequency set to: %.3f Hz' % self.sample_frequency)
        
        self.b0 = 0.0
        self.b1 = 0.0
        self.a1 = 0.0
        self.calculate_filter_coefficients()
        self.get_logger().info('Filter coefficients calculated: b0=%.3f, b1=%.3f, a1=%.3f' % (self.b0, self.b1, self.a1))
        
        self.prev_raw_data = Imu()
        self.prev_data = Imu()

        self.prev_raw_data = self.set_default_values(self.prev_raw_data)
        self.prev_data = self.set_default_values(self.prev_data)
        self.get_logger().info('Previous data initialized to zero values.')

        self.timer = self.create_timer(timer_period, self.timer_callback)   


    def timer_callback(self):
        raw_data = Imu()
        data = Imu()

        accel = None
        gyro = None

        elapsed = time.time() - self.time_
        sec = int(elapsed)
        nanosec = int((elapsed - sec) * 1e9)

        if nanosec < 0:
            nanosec = 0
    
        try:
            accel = navigator.read_accel()
            gyro = navigator.read_gyro()
        except Exception as err:
            self.get_logger().error(str(err))

        if accel is not None and gyro is not None:
            raw_data = self.set_data(raw_data, accel, gyro)
            data = self.low_pass_filter(raw_data)
            raw_data = self.correct_coordinates(raw_data)
            data = self.correct_coordinates(data)

            self.get_logger().info('Current coordinates: Linear[%.3f, %.3f, %.3f], Angular[%.3f, %.3f, %.3f]' %
                (self.coordinate_system_linear[0], self.coordinate_system_linear[1], self.coordinate_system_linear[2],
                self.coordinate_system_angular[0], self.coordinate_system_angular[1], self.coordinate_system_angular[2]))
            self.get_logger().info('Publishing raw data: Accel[%.3f, %.3f, %.3f], Gyro[%.3f, %.3f, %.3f]' %
                                   (accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z))
        else:
            
            raw_data = self.set_default_values(raw_data)
            data = self.set_default_values(data)
            
            self.get_logger().info('Publishing default values: Accel[0, 0, 0], Gyro[0, 0, 0]')
            

        raw_data = self.set_header(raw_data, 'imu_link', sec, nanosec)
        data = self.set_header(data, 'imu_link', sec, nanosec)

        self.rawDataPublisher_.publish(raw_data)
        self.dataPublisher.publish(data)
        
    def low_pass_filter(self, raw_data):
        data = Imu()
        data.linear_acceleration.x = self.low_pass_filter_single_axis(raw_data.linear_acceleration.x,
                                                                     self.prev_raw_data.linear_acceleration.x,
                                                                     self.prev_data.linear_acceleration.x)
        data.linear_acceleration.y = self.low_pass_filter_single_axis(raw_data.linear_acceleration.y,
                                                                     self.prev_raw_data.linear_acceleration.y,
                                                                     self.prev_data.linear_acceleration.y)
        data.linear_acceleration.z = self.low_pass_filter_single_axis(raw_data.linear_acceleration.z,
                                                                     self.prev_raw_data.linear_acceleration.z,
                                                                     self.prev_data.linear_acceleration.z)
        data.angular_velocity.x = self.low_pass_filter_single_axis(raw_data.angular_velocity.x,
                                                                  self.prev_raw_data.angular_velocity.x,
                                                                  self.prev_data.angular_velocity.x)
        data.angular_velocity.y = self.low_pass_filter_single_axis(raw_data.angular_velocity.y,
                                                                  self.prev_raw_data.angular_velocity.y,
                                                                  self.prev_data.angular_velocity.y)   
        data.angular_velocity.z = self.low_pass_filter_single_axis(raw_data.angular_velocity.z,
                                                                  self.prev_raw_data.angular_velocity.z,
                                                                  self.prev_data.angular_velocity.z)     
        self.prev_raw_data = raw_data
        self.prev_data = data
        
        return data
    
    def low_pass_filter_single_axis(self, raw_value, prev_raw_value, prev_value):
        value = self.b0 * raw_value + self.b1 * prev_raw_value + self.a1 * prev_value
        return value
    
    def set_header(self, data, frame_id, sec, nanosec):
        data.header.frame_id = frame_id
        data.header.stamp.sec = sec
        data.header.stamp.nanosec = nanosec
        return data
    
    def set_default_values(self, data):
        data.linear_acceleration.x = 0.0
        data.linear_acceleration.y = 0.0
        data.linear_acceleration.z = 0.0
        data.angular_velocity.x = 0.0
        data.angular_velocity.y = 0.0
        data.angular_velocity.z = 0.0
        return data
    
    def set_data(self, data, accel, gyro):
        data.linear_acceleration.x = accel.x
        data.linear_acceleration.y = accel.y
        data.linear_acceleration.z = accel.z
        data.angular_velocity.x = gyro.x
        data.angular_velocity.y = gyro.y
        data.angular_velocity.z = gyro.z
        return data

    def calculate_filter_coefficients(self):
        # K = math.tan(math.pi * self.cutoff_frequency / self.sample_frequency)
        # self.b0 = K / (1 + K)
        # self.b1 = self.b0
        # self.a1 = (K - 1) / (1 + K)

        K = (self.cutoff_frequency / self.sample_frequency)/2
        self.b0 = K / (1 + K)
        self.b1 = self.b0
        self.a1 = (1 - K) / (1 + K)
   
    def correct_coordinates(self, data):
        data.linear_acceleration.x = data.linear_acceleration.x * self.coordinate_system_linear[0]
        data.linear_acceleration.y = data.linear_acceleration.y * self.coordinate_system_linear[1]
        data.linear_acceleration.z = data.linear_acceleration.z * self.coordinate_system_linear[2]
        data.angular_velocity.x = data.angular_velocity.x * self.coordinate_system_angular[0]
        data.angular_velocity.y = data.angular_velocity.y * self.coordinate_system_angular[1]
        data.angular_velocity.z = data.angular_velocity.z * self.coordinate_system_angular[2]
        return data


def main(args=None):
    rclpy.init(args=args)

    imu_driver = ImuPublisher()

    rclpy.spin(imu_driver)

    imu_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
