import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

# Dummy GPIO class for simulation
class DummyGPIO:
    OUT = 'out'
    HIGH = 'high'
    LOW = 'low'

    @staticmethod
    def setmode(mode):
        pass

    @staticmethod
    def setup(pin, mode):
        pass

    @staticmethod
    def output(pin, state):
        print(f"GPIO Pin {pin} set to {state}")

    @staticmethod
    def cleanup():
        print("Cleaning up GPIO")

# Replace 'DummyGPIO' with 'RPi.GPIO as GPIO' in actual implementation
GPIO = DummyGPIO

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.subscription = self.create_subscription(
            Joy,
            '/joy',  # Subscribing to the /joy topic
            self.joystick_callback,
            10)
        self.subscription

        # Set up GPIO (for simulation, replace with actual setup in implementation)
        GPIO.setmode(GPIO.OUT)
        self.IN1 = 17  # Example GPIO pin numbers
        self.IN2 = 27
        self.IN3 = 22
        self.IN4 = 23
        GPIO.setup([self.IN1, self.IN2, self.IN3, self.IN4], GPIO.OUT)

    def joystick_callback(self, msg):
        # Axes[4] is the vertical axis of the right stick for forward/backward
        # Axes[0] is the horizontal axis of the left stick for turning

        forward_backward = msg.axes[4]  # Right stick vertical
        turning = msg.axes[0]  # Left stick horizontal

        # Determine motor speeds based on joystick input
        left_motor_speed = forward_backward + turning
        right_motor_speed = forward_backward - turning

        # Log the motor speeds for visualization
        self.get_logger().info(f"Left Motor Speed: {left_motor_speed}, Right Motor Speed: {right_motor_speed}")

        # Dummy GPIO control logic (replace with actual logic in implementation)
        GPIO.output(self.IN1, GPIO.HIGH if left_motor_speed > 0 else GPIO.LOW)
        GPIO.output(self.IN2, GPIO.LOW if left_motor_speed > 0 else GPIO.HIGH)
        GPIO.output(self.IN3, GPIO.HIGH if right_motor_speed > 0 else GPIO.LOW)
        GPIO.output(self.IN4, GPIO.LOW if right_motor_speed > 0 else GPIO.HIGH)


def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    rclpy.spin(motor_controller)
    GPIO.cleanup()
    motor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

