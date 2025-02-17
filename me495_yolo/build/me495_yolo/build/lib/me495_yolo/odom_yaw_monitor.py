import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import tf_transformations
import math

class OdomYawMonitor(Node):
    def __init__(self):
        super().__init__('odom_yaw_monitor')

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

    def odom_callback(self, msg):
        """Extract yaw from odometry quaternion."""
        quat = msg.pose.pose.orientation
        (_, _, yaw) = tf_transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        
        yaw_deg = math.degrees(yaw)  # Convert to degrees for easier interpretation
        self.get_logger().info(f"🔄 TurtleBot Yaw: {yaw_deg:.2f}°")

def main():
    rclpy.init()
    node = OdomYawMonitor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
