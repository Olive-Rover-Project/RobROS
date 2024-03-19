import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Quaternion

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')
        # Primary GPS Publisher (for rover)
        self.gps_publisher = self.create_publisher(NavSatFix, 'gps/fix', 10)
        # Secondary GPS Publisher (for goal)
        self.gps_publisher_goal = self.create_publisher(NavSatFix, 'gps/goal', 10)  # Goal GPS publisher
        
        # IMU Publisher
        self.imu_publisher = self.create_publisher(Imu, 'imu/data', 10)
        
        self.timer = self.create_timer(1.0, self.publish_sensor_data)
        
        # Rover GPS data
        self.latitude = 40.0  # Starting latitude, change as needed
        self.longitude = -74.0  # Starting longitude, change as needed
        self.altitude = 100.0  # Starting altitude, change as needed
        
        # Goal GPS data (constant)
        self.goal_latitude = 40.001  # Example goal latitude
        self.goal_longitude = -74.001  # Example goal longitude
        self.goal_altitude = 100.0  # Example goal altitude
        
        # IMU data (example values, adjust as needed)
        self.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)  # Neutral orientation
        self.angular_velocity = (0.0, 0.0, 0.0)  # Static conditions
        self.linear_acceleration = (0.0, 0.0, 0.0)  # Static conditions

    def publish_sensor_data(self):
        self.publish_gps_coords()
        self.publish_goal_gps_coords()  # Publish goal GPS coordinates
        self.publish_imu_data()

    def publish_gps_coords(self):
        gps_msg = NavSatFix()
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = "gps_frame"
        gps_msg.latitude = self.latitude
        gps_msg.longitude = self.longitude
        gps_msg.altitude = self.altitude
        
        # Increment GPS coordinates for simulation
        delta_deg = (0.5 / 111000)  # 50 cm in degrees
        self.latitude += delta_deg
        self.longitude -= delta_deg
        
        self.gps_publisher.publish(gps_msg)
        self.get_logger().info(f'Publishing Rover GPS Coords: Latitude {self.latitude}, Longitude {self.longitude}')

    def publish_goal_gps_coords(self):
        # This function publishes static goal GPS coordinates
        gps_msg_goal = NavSatFix()
        gps_msg_goal.header.stamp = self.get_clock().now().to_msg()
        gps_msg_goal.header.frame_id = "gps_frame_goal"  # Different frame_id for clarity
        gps_msg_goal.latitude = self.goal_latitude
        gps_msg_goal.longitude = self.goal_longitude
        gps_msg_goal.altitude = self.goal_altitude
        
        self.gps_publisher_goal.publish(gps_msg_goal)
        self.get_logger().info(f'Publishing Goal GPS Coords: Latitude {self.goal_latitude}, Longitude {self.goal_longitude}')

    def publish_imu_data(self):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_frame"
        imu_msg.orientation = self.orientation
        imu_msg.angular_velocity.x = self.angular_velocity[0]
        imu_msg.angular_velocity.y = self.angular_velocity[1]
        imu_msg.angular_velocity.z = self.angular_velocity[2]
        imu_msg.linear_acceleration.x = self.linear_acceleration[0]
        imu_msg.linear_acceleration.y = self.linear_acceleration[1]
        imu_msg.linear_acceleration.z = self.linear_acceleration[2]
        
        self.imu_publisher.publish(imu_msg)
        self.get_logger().info('Publishing IMU Data')

def main(args=None):
    rclpy.init(args=args)
    sensor_publisher = SensorPublisher()
    rclpy.spin(sensor_publisher)

if __name__ == '__main__':
    main()

