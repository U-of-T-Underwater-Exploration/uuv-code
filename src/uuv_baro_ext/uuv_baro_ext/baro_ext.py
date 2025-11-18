import rclpy
from rclpy.node import Node

from sensor_msgs.msg import FluidPressure, Temperature
from uuv_baro_ext import ms5837


class ExternalBarometerPublisher(Node):
    def __init__(self):
        super().__init__('external_barometer_publisher')

        self.declare_parameter("publish_rate", 0.2)

        self.sensor = ms5837.MS5837(model=ms5837.MODEL_30BA, bus=1)
        self.sensor.init()
        self.sensor.setFluidDensity(ms5837.DENSITY_FRESHWATER) #can be SALTWATER: 
        #Freshwater = 997kg/m^3, Saltwater = 1029kg/m^3

        self.filter_pressure_publisher = self.create_publisher(FluidPressure, 'baro/external/data', 10)
        self.raw_pressure_publisher = self.create_publisher(FluidPressure, 'baro/external/data_raw', 10)
        self.temperature_publisher = self.create_publisher(Temperature, 'baro/external/temperature', 10)

        self.timer_period = 1/self.get_parameter("publish_rate").get_parameter_value().double_value
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        #TODO filter data
        #self.sensor.read(ms5837.OSR_256) 
        #oversampling can be 256, 512, 1024, 2048, 4096, 8192

        filter_pressure_message = FluidPressure()
        filter_pressure_message.fluid_pressure = self.sensor.pressure(ms5837.UNITS_Pa)
        self.filter_pressure_publisher.publish(filter_pressure_message)

        raw_pressure_message = FluidPressure()
        raw_pressure_message.fluid_pressure = self.sensor.pressure(ms5837.UNITS_Pa)
        self.raw_pressure_publisher.publish(raw_pressure_message)

        temperature_message = Temperature()
        temperature_message.temperature = self.sensor.temperature(ms5837.UNITS_Centigrade)
        self.temperature_publisher.publish(temperature_message)



def main(args=None):
    rclpy.init(args=args)
    external_barometer_publisher = ExternalBarometerPublisher()
    rclpy.spin(external_barometer_publisher)

    external_barometer_publisher.destroy_node
    rclpy.shutdown()

if __name__ == '__main__':
    main()