import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import NavSatFix
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import utm

class GPSConverterNode(Node):
    def __init__(self):
        super().__init__('gps_converter_node')
        self.rover_gps_subscriber = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.rover_gps_callback,
            10)
        self.goal_gps_subscriber = self.create_subscription(
            NavSatFix,
            '/gps/goal',
            self.goal_gps_callback,
            10)
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal_pose_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        self.reference_utm = None
        self.get_logger().info("GPS Converter Node Started")

    def rover_gps_callback(self, msg):
        utm_coords = utm.from_latlon(msg.latitude, msg.longitude)
        if self.reference_utm is None:
            self.reference_utm = utm_coords
            self.get_logger().info(f"Set origin point: {self.reference_utm}")
        else:
            rover_offset_x = utm_coords[0] - self.reference_utm[0]
            rover_offset_y = utm_coords[1] - self.reference_utm[1]
            self.get_logger().info(f"Rover's distance from origin: x={rover_offset_x:.2f}m, y={rover_offset_y:.2f}m")

    def goal_gps_callback(self, msg):
        if self.reference_utm is None:
            self.get_logger().info("Waiting for initial rover GPS reading for reference...")
            return

        goal_utm = utm.from_latlon(msg.latitude, msg.longitude)
        goal_offset_x = goal_utm[0] - self.reference_utm[0]
        goal_offset_y = goal_utm[1] - self.reference_utm[1]
        self.get_logger().info(f"Goal's distance from origin: x={goal_offset_x:.2f}m, y={goal_offset_y:.2f}m")

        # Optionally, publish the goal pose for visualization or other purposes
        goal_pose_msg = PoseStamped()
        goal_pose_msg.header.frame_id = "map"
        goal_pose_msg.header.stamp = self.get_clock().now().to_msg()
        goal_pose_msg.pose.position.x = goal_offset_x
        goal_pose_msg.pose.position.y = goal_offset_y
        goal_pose_msg.pose.orientation.w = 1.0  # Assuming a default orientation
        self.goal_pose_publisher.publish(goal_pose_msg)
        # Send the goal to Nav2 using the action client
        self.send_goal_pose(goal_offset_x, goal_offset_y)

    def send_goal_pose(self, x, y):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0  # Facing forward
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        # You can also monitor for goal result here if needed


def main(args=None):
    rclpy.init(args=args)
    node = GPSConverterNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

