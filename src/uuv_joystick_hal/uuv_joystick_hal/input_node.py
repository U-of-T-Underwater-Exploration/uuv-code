#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from uuv_joystick_msgs.msg import UUVCommand, ActionCommand
from rclpy.qos import QoSProfile, ReliabilityPolicy

class JoystickHAL(Node):
    def __init__(self):
        super().__init__('joystick_hal')
        self.prev_buttons = []
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            qos
        )
        self.command_pub = self.create_publisher(
            UUVCommand,
            '/input/command',
            qos
        )

        self.get_logger().info('Joystick HAL node started.')

    def joy_callback(self, msg: Joy):
        try:
            cmd = UUVCommand()

            # Safe axis access
            cmd.surge = msg.axes[1] if len(msg.axes) > 1 else 0.0
            cmd.roll  = msg.axes[0] if len(msg.axes) > 0 else 0.0
            cmd.sway = msg.axes[3] if len(msg.axes) > 3 else 0.0
            cmd.heave = msg.axes[2] if len(msg.axes) > 2 else 0.0
            cmd.pitch  = msg.axes[4] if len(msg.axes) > 4 else 0.0   
            cmd.yaw = msg.axes[5] if len(msg.axes) > 5 else 0.0
            cmd.mode  = 0
            
            if not self.prev_buttons:
                self.prev_buttons = [0] * len(msg.buttons)
            #AXIS_ONLY_BUTTONS = {4, 5}  # Only want the RB/LB to be used as axis not buttons
            #Corresponding # of button bo be confirmed based on joystick use #4 and #5 for now
            
            cmd.actions = []
            for i, pressed in enumerate(msg.buttons):
                """
                if i in AXIS_ONLY_BUTTONS:
                    continue
                """
                prev = self.prev_buttons[i]

                a = ActionCommand()
                a.action = i

                if pressed and not prev:
                    a.state = ActionCommand().PRESSED
                elif not pressed and prev:
                    a.state = ActionCommand().RELEASED
                elif pressed and prev:
                    a.state = ActionCommand().HELD
                else:
                    a.state = ActionCommand().INACTIVE
                if a.state != ActionCommand().INACTIVE:
                    cmd.actions.append(a)
            self.prev_buttons = list(msg.buttons)
            #self.get_logger().info("Joystick message received")
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
