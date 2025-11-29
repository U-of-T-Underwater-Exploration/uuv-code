import rclpy
from rclpy.node import Node
try:
    import bluerobotics_navigator as navigator
except Exception:
    navigator = None
import yaml
import os
import time 

from sensor_msgs.msg import MagneticField
from builtin_interfaces.msg import Time


class CompassPublisher(Node):

    def __init__(self):
        super().__init__('compass_publisher')
        self.rawDataPublisher_ = self.create_publisher(MagneticField, 'compass/data_raw', 10)
        self.dataPublisher = self.create_publisher(MagneticField, 'compass/data', 10)

        self.time_ = time.time()
        
        #From IMU template
        # tbd: change the path to config file to be dynamic
        #with open("/home/katherine/Documents/GitHub/uuv-code/src/uuv_imu_driver/uuv_imu_driver/uuv_imu_driver.yml") as file:
        #    config = yaml.safe_load(file)
        
        timer_period = 0.1 #config['timer_period']

        try: navigator.init()
        except:
            self.get_logger().error('Failed to initialize connection to Navigator')


        self.timer = self.create_timer(timer_period, self.timer_callback)   


    def timer_callback(self):
        raw_data = MagneticField()
        data = MagneticField()

        # accel = None
        # gyro = None
        magfield = None

        elapsed = time.time() - self.time_
        sec = int(elapsed)
        nanosec = int((elapsed - sec) * 1e9)

        if nanosec < 0:
            nanosec = 0
    
        

        #try: 
        magfield = navigator.read_mag()
        # gyro = navigator.read_gyro()
        #except:
        #   self.get_logger().error('Failed to get IMU data from Navigator')

        if magfield is not None:
            raw_data.magnetic_field.x = magfield.x
            raw_data.magnetic_field.y = magfield.y
            raw_data.magnetic_field.z = magfield.z

            # raw_data.angular_velocity.x = gyro.x
            # raw_data.angular_velocity.y = gyro.y
            # raw_data.angular_velocity.z = gyro.z

            data = self.low_pass_filter(raw_data)

            self.get_logger().info('Publishing raw data: MagField[%.3f, %.3f, %.3f]' %
                                   (magfield.x, magfield.y, magfield.z))
        else:
            
            raw_data.magnetic_field.x = data.magnetic_field.x = float(0)
            raw_data.magnetic_field.y = data.magnetic_field.y = float(0)
            raw_data.magnetic_field.z = data.magnetic_field.z = float(0)
            
            self.get_logger().info('Publishing default values: MagField[0, 0, 0]')
            
        raw_data.header.frame_id = 'compass_link'
        raw_data.header.stamp.nanosec = nanosec
        raw_data.header.stamp.sec = sec

        data.header.frame_id = 'compass_link'
        data.header.stamp.nanosec = nanosec
        data.header.stamp.sec = sec
        
        self.rawDataPublisher_.publish(raw_data)
        self.dataPublisher.publish(data)
        
    def low_pass_filter(self, raw_data):
        return raw_data  # Placeholder for actual low-pass filter implementation

def main(args=None):
    rclpy.init(args=args)

    compass_driver = CompassPublisher()

    rclpy.spin(compass_driver)

    compass_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
