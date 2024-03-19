import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class JoystickPublisher(Node):
    def __init__(self):
        super().__init__('joystick_publisher')
        self.publisher_ = self.create_publisher(Joy, 'joystick_data', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Joy()
        # Add joystick data to msg here
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = [0.0, 1.0]  # Example data
        msg.buttons = [0, 1, 0]  # Example data
        self.publisher_.publish(msg)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    joystick_publisher = JoystickPublisher()
    rclpy.spin(joystick_publisher)
    joystick_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
