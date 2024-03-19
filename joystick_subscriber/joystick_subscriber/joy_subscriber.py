import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class JoystickSubscriber(Node):
    def __init__(self):
        super().__init__('joystick_subscriber')
        self.subscription = self.create_subscription(
            Joy,
            'joystick_data',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        rounded_axes = [round(axis, 2) for axis in msg.axes]  # 2 decimals
        self.get_logger().info(f'Rounded Axes: {rounded_axes}, Buttons: {msg.buttons}')

def main(args=None):
    rclpy.init(args=args)
    joystick_subscriber = JoystickSubscriber()
    rclpy.spin(joystick_subscriber)
    joystick_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

