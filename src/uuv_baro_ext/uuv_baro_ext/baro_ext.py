import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from sensor_msgs.msg import FluidPressure, Temperature
import ms5837
import time


class ExternalBarometerPublisher(Node):
    def __init__(self):
        super().__init__('external_barometer_publisher')

        '''
        qos_profile = QoSProfile(
            reliability = QoSReliabilityPolicy.RELIABLE
        )
        '''

        self.declare_parameter("publish_rate", 0.2)
        self.declare_parameter("frame_id", "default_frame")

        # LPF parameters
        self.declare_parameter("cutoff_frequency", 0.2) # Hz
        self.declare_parameter("sampling_frequency", 50.0) # Hz

        #start_time = self.get_clock().now()

        self.sensor = ms5837.MS5837(ms5837.MODEL_30BA, 6)
        if not self.sensor.init():
            self.get_logger().error("Sensor could not be initialized")

        self.sensor.setFluidDensity(ms5837.DENSITY_FRESHWATER) #can be SALTWATER: 
        #Freshwater = 997kg/m^3, Saltwater = 1029kg/m^3

        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.raw_pressure_publisher = self.create_publisher(FluidPressure, 'baro/external/data_raw', 10)
        self.filter_pressure_publisher = self.create_publisher(FluidPressure, 'baro/external/data', 10)

        self.raw_temperature_publisher = self.create_publisher(Temperature, 'baro/external/temperature_raw', 10)
        self.filter_temperature_publisher = self.create_publisher(Temperature, 'baro/external/temperature', 10)

        self.timer_period = 1/self.get_parameter("publish_rate").get_parameter_value().double_value
        self.timer = self.create_timer(self.timer_period, self.timer_callback)


        cut_freq = self.get_parameter('cutoff_frequency').value
        samp_freq = self.get_parameter('sampling_frequency').value
        delta_time = 1 / samp_freq

        self.alpha = (2 * 3.14159 * cut_freq * delta_time) / (2 * 3.14159 * cut_freq * delta_time + 1)

        self.last_filtered_pressure_data = None
        self.last_filtered_temperature_data = None


    def timer_callback(self):

        try:
            status = self.sensor.read()
            if not status:
                self.get_logger().warning("Sensor read failed")
                return
                
            read_time = self.get_clock().now().to_msg()

            raw_pressure_data = self.sensor.pressure(ms5837.UNITS_Pa)
            filtered_pressure_data = self.low_pass_filter(raw_pressure_data, self.last_filtered_pressure_data)

            raw_temperature_data = self.sensor.temperature(ms5837.UNITS_Centigrade)
            filtered_temperature_data = self.low_pass_filter(raw_temperature_data, self.last_filtered_temperature_data)


            raw_pressure_message = FluidPressure()
            raw_pressure_message.header.frame_id = self.frame_id
            raw_pressure_message.header.stamp = read_time
            raw_pressure_message.fluid_pressure = raw_pressure_data
            self.raw_pressure_publisher.publish(raw_pressure_message)


            filter_pressure_message = FluidPressure()
            filter_pressure_message.header.frame_id = self.frame_id
            filter_pressure_message.header.stamp = read_time
            filter_pressure_message.fluid_pressure = filtered_pressure_data
            self.last_filtered_pressure_data = filtered_pressure_data
            self.filter_pressure_publisher.publish(filter_pressure_message)


            raw_temperature_message = Temperature()
            raw_temperature_message.header.frame_id = self.frame_id
            raw_temperature_message.header.stamp = read_time
            raw_temperature_message.temperature = float(raw_temperature_data)
            self.raw_temperature_publisher.publish(raw_temperature_message)


            filter_temperature_message = Temperature()
            filter_temperature_message.header.frame_id = self.frame_id
            filter_temperature_message.header.stamp = read_time
            filter_temperature_message.temperature = filtered_temperature_data
            self.last_filtered_temperature_data = filtered_temperature_data
            self.filter_temperature_publisher.publish(filter_temperature_message)

        except Exception as e:
            self.get_logger().error(f"Exception in timer_callback: {e}")
            return

    def low_pass_filter(self, current_raw_data, last_filtered_data):
        if last_filtered_data == None:
            return current_raw_data
        
        filtered_data = (self.alpha * current_raw_data) + \
                        ((1.0 - self.alpha) * last_filtered_data)

        return filtered_data

def main(args=None):
    rclpy.init(args=args)
    external_barometer_publisher = ExternalBarometerPublisher()
    rclpy.spin(external_barometer_publisher)

    external_barometer_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()