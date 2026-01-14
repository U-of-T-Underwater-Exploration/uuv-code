import rclpy
from rclpy.node import Node
try:
    import bluerobotics_navigator as navigator
except Exception:
    navigator = None
import yaml
import os
import time 

from sensor_msgs.msg import Imu
from builtin_interfaces.msg import Time


class ImuPublisher(Node):

    def __init__(self):
        super().__init__('imu_publisher')
        self.rawDataPublisher_ = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.dataPublisher = self.create_publisher(Imu, 'imu/data', 10)
        self.declare_parameter('timer_period', 0.5)  # seconds

        self.time_ = time.time()
        
        timer_period = self.get_parameter('timer_period').get_parameter_value().double_value
        self.get_logger().info('Timer period set to: %.3f seconds' % timer_period)

        try: navigator.init()
        except Exception as err:
            self.get_logger().error(str(err))


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
        return raw_data  # Placeholder for actual low-pass filter implementation
    
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

def main(args=None):
    rclpy.init(args=args)

    imu_driver = ImuPublisher()

    rclpy.spin(imu_driver)

    imu_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
