import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import numpy as np
import utm

class InitialPositionAverager(Node):
    def __init__(self):
        super().__init__('initial_position_averager')
        self.gps_subscriber = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            10)
        self.positions = []  # Store UTM coordinates
        self.max_readings = 10

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
        self.get_logger().info(f'Average initial position in UTM: Easting {avg_easting}, Northing {avg_northing}')

def main(args=None):
    rclpy.init(args=args)
    averager = InitialPositionAverager()
    rclpy.spin(averager)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

