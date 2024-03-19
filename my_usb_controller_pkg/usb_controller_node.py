#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class USBControllerNode(Node):
    def __init__(self):
        super().__init__('usb_controller_node')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)
        self.subscription  # prevent unused variable warning

    def joy_callback(self, msg):
        self.get_logger().info('Received Joy data: "%s"' % msg)

def main(args=None):
    rclpy.init(args=args)
    usb_controller_node = USBControllerNode()
    rclpy.spin(usb_controller_node)
    usb_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
