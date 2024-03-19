import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf2_ros
import tf2_geometry_msgs
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
import math

class IncrementalNavigator(Node):
    def __init__(self):
        super().__init__('incremental_navigator')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.subscription_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        self.goal_pose = self.get_initial_goal()  # Set your initial goal here

    def get_initial_goal(self):
        # Define your method to set the initial goal pose
        # This could be a static position or derived from some conditions
        goal_pose = TransformStamped()
        goal_pose.transform.translation.x = 1.0  # Example goal, 1 meter ahead
        goal_pose.transform.translation.y = 0.0
        goal_pose.transform.translation.z = 0.0
        goal_pose.transform.rotation.w = 1.0  # No rotation
        return goal_pose

    def get_current_pose(self):
        try:
            # Ensure the frame names match your robot's TF tree
            current_pose = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            return current_pose
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error('Could not transform base_link to map: %s' % str(e))
            return None

    def scan_callback(self, msg):
        # Process LIDAR scans to detect obstacles and adjust the path if necessary
        pass

    def navigate_to_goal(self):
        # Example logic to move towards the goal
        current_pose = self.get_current_pose()
        if current_pose is not None and self.goal_pose is not None:
            # Implement your navigation logic here
            # This is a placeholder for moving forward
            twist_msg = Twist()
            twist_msg.linear.x = 0.1  # Move forward
            self.publisher_.publish(twist_msg)

    def update_goal(self):
        # Logic to update the goal pose incrementally
        pass

def main(args=None):
    rclpy.init(args=args)
    incremental_navigator = IncrementalNavigator()

    # Use a timer to periodically call navigate_to_goal
    timer_period = 1.0  # seconds
    navigator_timer = incremental_navigator.create_timer(timer_period, incremental_navigator.navigate_to_goal)

    rclpy.spin(incremental_navigator)
    incremental_navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

