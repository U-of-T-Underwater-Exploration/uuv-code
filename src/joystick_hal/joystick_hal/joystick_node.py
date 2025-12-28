import rclpy
import yaml
import os
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy



class JoystickHAL(Node):
    def __init__(self):
        Node.__init__(self, 'joystick_hal')
        config_path = os.path.join(
            os.path.dirname(__file__),
            '..',
            'config',
            'button_mapping.yaml'
        )

        try:
            with open(config_path, 'r') as f:
                self.button_map = yaml.safe_load(f).get('buttons', {})
                self.get_logger().info(f'Loaded button map: {self.button_map}')
        except Exception as e:
            self.get_logger().error(f'Failed to load button mapping: {e}')
            self.button_map = {}
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            qos
        )
        self.command_pub = self.create_publisher(
            String,
            'rov/joystick',
            qos
        )

        self.get_logger().info('Joystick HAL node started.')

    def joy_callback(self, msg: Joy):
        try:
            for command, button_index in self.button_map.items():
                #only support buttons so far, axis control haven't been added
                if button_index < len(msg.buttons) and msg.buttons[button_index] == 1:
                    out = String()
                    out.data = command
                    self.command_pub.publish(out)
                    self.get_logger().info(f'Published command: {command}')
        except Exception as e:
            self.get_logger().error(f'Error processing joystick input: {e}')



def main(args=None):
    rclpy.init(args=args)
    node = JoystickHAL()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
