#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from joystick_hal.msg import UUVCommand, ActionCommand
from rclpy.qos import QoSProfile, ReliabilityPolicy



class JoystickHAL(Node):
    def __init__(self):
        super().__init__('joystick_hal')

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
            UUVCommand,
            '/input',
            qos
        )

        self.get_logger().info('Joystick HAL node started.')

    def joy_callback(self, msg: Joy):
        try:
            cmd = UUVCommand()

            # Safe axis access
            cmd.surge = msg.axes[1] if len(msg.axes) > 1 else 0.0
            cmd.sway  = msg.axes[0] if len(msg.axes) > 0 else 0.0
            cmd.heave = msg.axes[3] if len(msg.axes) > 3 else 0.0
            cmd.yaw   = msg.axes[2] if len(msg.axes) > 2 else 0.0
            cmd.roll  = 0.0
            cmd.pitch = 0.0
            cmd.mode  = 0

            cmd.actions = []
            for i, pressed in enumerate(msg.buttons):
                if pressed:
                    a = ActionCommand()
                    a.action = i
                    a.state = 1
                    cmd.actions.append(a)
            self.get_logger().info("Joystick message received")
            self.command_pub.publish(cmd)

        except Exception as e:
            self.get_logger().error(f'Joystick processing failed: {e}')




def main(args=None):
    rclpy.init(args=args)
    node = JoystickHAL()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
