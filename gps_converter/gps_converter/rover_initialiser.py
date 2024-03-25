import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import utm

class InitialPositionAverager(Node):
    def __init__(self):
        super().__init__('rover_initialiser')
        self.gps_subscriber = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            10)
        self.initialpose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10)
        self.positions = []  # Store UTM coordinates
        self.max_readings = 10
        self.map_origin_utm = (412022.624244104, 6182128.56978424)  # Example map origin in UTM

    def gps_callback(self, msg):
        # Convert Rover GPS lat/lon to UTM coordinates
        utm_coords = utm.from_latlon(msg.latitude, msg.longitude)
        # Add UTM coordinates (easting, northing) to the positions list
        self.positions.append((utm_coords[0], utm_coords[1]))

        if len(self.positions) == self.max_readings:
            self.compute_average_position()

    def compute_average_position(self):
        # Compute average UTM coordinates
        avg_easting = np.mean([pos[0] for pos in self.positions])
        avg_northing = np.mean([pos[1] for pos in self.positions])

        # Compute offset from map origin
        offset_x = avg_easting - self.map_origin_utm[0]
        offset_y = avg_northing - self.map_origin_utm[1]

        # Assume facing north
        orientation_theta = 0.0 

        self.publish_initial_pose(offset_x, offset_y, orientation_theta)

        self.get_logger().info(f'Average initial position in UTM: Easting {avg_easting}, Northing {avg_northing}')

    def publish_initial_pose(self, x, y, theta):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        #For facing north
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0

        self.initialpose_publisher.publish(msg)
        self.get_logger().info('Publishing initial pose')

def main(args=None):
    rclpy.init(args=args)
    averager = InitialPositionAverager()
    rclpy.spin(averager)
    averager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

