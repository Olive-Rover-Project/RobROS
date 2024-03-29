#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')
        self.publisher_ = self.create_publisher(String, '/test_topic', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %s' % self.get_clock().now().to_msg()
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    test_publisher = TestPublisher()
    rclpy.spin(test_publisher)
    test_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
