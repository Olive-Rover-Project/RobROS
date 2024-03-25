import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import time

class GPSPublisher(Node):
    def __init__(self):
        super().__init__('gps_publisher')
        self.publisher = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.coords = [
            (55.77694702, -4.39883423),	#Just manually doing this
            (55.77693176, -4.39885139),
            (55.77696228, -4.39887810),
            (55.77696228, -4.39888954),
            (55.77696228, -4.39889336),
            (55.77694702, -4.39890671),
            (55.77694702, -4.39892387),
            (55.77694702, -4.39894485),
            (55.77696228, -4.39896202),
            (55.77696228, -4.39899254),
            (55.77697754, -4.39900589)
        ]
        self.timer = self.create_timer(1.0, self.publish_coords)
        self.index = 0

    def publish_coords(self):
        if self.index < len(self.coords):
            lat, lon = self.coords[self.index]
            msg = NavSatFix()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'gps_frame'
            msg.latitude = lat
            msg.longitude = lon
            self.publisher.publish(msg)
            self.get_logger().info(f'Published GPS Fix: {lat}, {lon}')
            self.index += 1
        else:
            self.index = 0  # Reset index to loop through coordinates

def main(args=None):
    rclpy.init(args=args)
    gps_publisher = GPSPublisher()
    rclpy.spin(gps_publisher)
    gps_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

