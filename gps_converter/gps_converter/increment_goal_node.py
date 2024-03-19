import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import PoseStamped

class MapPoseListener(Node):
    def __init__(self):
        super().__init__('map_pose_listener')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.get_logger().info("Listening for the robot's pose in the map frame")

    def get_current_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            pose.pose.position.x = trans.transform.translation.x
            pose.pose.position.y = trans.transform.translation.y
            pose.pose.orientation = trans.transform.rotation

            self.get_logger().info(f"Current Pose in Map: {pose}")
        except Exception as e:
            self.get_logger().error('Failed to find pose in map frame: %r' % (e,))

def main(args=None):
    rclpy.init(args=args)
    node = MapPoseListener()
    rclpy.spin_once(node)
    node.get_current_pose()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

