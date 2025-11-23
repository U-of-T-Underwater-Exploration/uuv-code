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

        self.time_ = time.time()
        
        # tbd: change the path to config file to be dynamic
        #with open("/home/katherine/Documents/GitHub/uuv-code/src/uuv_imu_driver/uuv_imu_driver/uuv_imu_driver.yml") as file:
        #    config = yaml.safe_load(file)
        
        timer_period = 0.1 #config['timer_period']

        try: navigator.init()
        except:
            self.get_logger().error('Failed to initialize connection to Navigator')


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
            nonosec = 0
    
        

        try: 
            accel = navigator.get_accel()
            gyro = navigator.get_gyro()
        except:
            self.get_logger().error('Failed to get IMU data from Navigator')

        if accel is not None and gyro is not None:
            raw_data.linear_acceleration.x = accel.x
            raw_data.linear_acceleration.y = accel.y
            raw_data.linear_acceleration.z = accel.z

            raw_data.angular_velocity.x = gyro.x
            raw_data.angular_velocity.y = gyro.y
            raw_data.angular_velocity.z = gyro.z

            data = self.low_pass_filter(raw_data)

            self.get_logger().info('Publishing raw data: Accel[%.3f, %.3f, %.3f], Gyro[%.3f, %.3f, %.3f]' %
                                   (accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z))
        else:
            
            raw_data.linear_acceleration.x = data.linear_acceleration.x = float(0)
            raw_data.linear_acceleration.y = data.linear_acceleration.y = float(0)
            raw_data.linear_acceleration.z = data.linear_acceleration.z = float(0)

            raw_data.angular_velocity.x = data.angular_velocity.x = float(0)
            raw_data.angular_velocity.y = data.angular_velocity.y = float(0)
            raw_data.angular_velocity.z = data.angular_velocity.z = float(0)
            
            self.get_logger().info('Publishing default values: Accel[0, 0, 0], Gyro[0, 0, 0]')
            
        raw_data.header.frame_id = 'imu_link'
        raw_data.header.stamp.nanosec = nanosec
        raw_data.header.stamp.sec = sec

        data.header.frame_id = 'imu_link'
        data.header.stamp.nanosec = nanosec
        data.header.stamp.sec = sec
        
        self.rawDataPublisher_.publish(raw_data)
        self.dataPublisher.publish(data)
        
    def low_pass_filter(self, raw_data):
        return raw_data  # Placeholder for actual low-pass filter implementation

def main(args=None):
    rclpy.init(args=args)

    imu_driver = ImuPublisher()

    rclpy.spin(imu_driver)

    imu_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
