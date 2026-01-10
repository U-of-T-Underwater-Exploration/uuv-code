import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField
import numpy as np
import time


class MagCalibrator(Node):
    def __init__(self):
        super().__init__('mag_calibrator')

        # Parameters
        self.declare_parameter('calibration_duration', 10.0)  # seconds
        self.declare_parameter('reference_field', [0.0, 0.0, 0.0])  # μT or Tesla

        self.calibration_duration = self.get_parameter(
            'calibration_duration').value
        self.reference_field = np.array(
            self.get_parameter('reference_field').value)

        # Subscribers & publishers
        self.sub = self.create_subscription(
            MagneticField,
            'compass/data_raw',
            self.mag_callback,
            10)

        self.pub = self.create_publisher(
            MagneticField,
            'compass/data_corrected',
            10)

        # Calibration state
        self.calibrating = True
        self.start_time = time.time()
        self.samples = []

        self.bias = np.zeros(3)
        self.correction = np.zeros(3)

        self.get_logger().info(
            f"Mag calibration started for {self.calibration_duration}s")

    def mag_callback(self, msg):
        B = np.array([
            msg.magnetic_field.x,
            msg.magnetic_field.y,
            msg.magnetic_field.z
        ])

        # --- Calibration phase ---
        if self.calibrating:
            self.samples.append(B)

            if time.time() - self.start_time >= self.calibration_duration:
                self.finish_calibration()
            return

        # --- Correction phase ---
        B_corrected = B + self.correction

        corrected_msg = MagneticField()
        corrected_msg.header = msg.header
        corrected_msg.magnetic_field.x = B_corrected[0]
        corrected_msg.magnetic_field.y = B_corrected[1]
        corrected_msg.magnetic_field.z = B_corrected[2]
        corrected_msg.magnetic_field_covariance = msg.magnetic_field_covariance

        self.pub.publish(corrected_msg)

    def finish_calibration(self):
        self.samples = np.array(self.samples)
        self.bias = np.mean(self.samples, axis=0)

        self.correction = self.reference_field - self.bias
        self.calibrating = False

        self.get_logger().info("Calibration complete")
        self.get_logger().info(f"Bias b = {self.bias}")
        self.get_logger().info(f"Correction e = {self.correction}")


def main(args=None):
    rclpy.init(args=args)
    node = MagCalibrator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


