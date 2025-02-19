# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from std_msgs.msg import Float32MultiArray

# class RotateTo90(Node):
#     def __init__(self):
#         super().__init__('rotate_to_90')
#         self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.create_subscription(Float32MultiArray, '/turtlebot_orientation', self.orientation_callback, 10)
#         self.yaw = None
#         self.previous_yaw = None  # Store last valid yaw to detect jumps
#         self.timer = self.create_timer(0.1, self.control_loop)  # Check every 100ms

#     def orientation_callback(self, msg):
#         if msg.data:
#             new_yaw = msg.data[0] % 360  # Keep yaw within [0, 360]

#             # Detect and handle sudden yaw jumps
#             if self.previous_yaw is not None:
#                 yaw_jump = abs(new_yaw - self.previous_yaw)
#                 if yaw_jump > 200:  # Large jump detected
#                     self.get_logger().warn(f"⚠️ Yaw jump detected: {self.previous_yaw}° → {new_yaw}°")
#                     new_yaw = self.previous_yaw  # Ignore the jump

#             self.previous_yaw = new_yaw  # Store last valid yaw
#             self.yaw = new_yaw

#     def shortest_angular_distance(self, target, current):
#         """Compute the shortest angular distance (in degrees) between current and target."""
#         diff = (target - current + 180) % 360 - 180  # Normalize between [-180, 180]
#         return diff

#     def control_loop(self):
#         if self.yaw is None:
#             self.get_logger().info("Waiting for orientation data...")
#             return

#         yaw_error = self.shortest_angular_distance(90, self.yaw)  # Compute error to target
#         self.get_logger().info(f"📏 Current Yaw: {self.yaw:.2f}° (Target: 90°), Error: {yaw_error:.2f}°")

#         twist = Twist()
#         if abs(yaw_error) > 5:  # Keep turning if error is significant
#             twist.angular.z = 0.3 if yaw_error > 0 else -0.3  # Rotate in correct direction
#             self.get_logger().info(f"🔄 Turning... Error: {yaw_error:.2f}°")
#         else:
#             self.get_logger().info("✅ Reached 90 degrees! Stopping.")
#             twist.angular.z = 0.0  # Stop turning
#             for _ in range(5):  # Send multiple stop commands for safety
#                 self.cmd_vel_pub.publish(twist)

#             self.destroy_node()

#         self.cmd_vel_pub.publish(twist)

# def main():
#     rclpy.init()
#     node = RotateTo90()
#     rclpy.spin(node)
#     rclpy.shutdown()

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

class RotateTo90(Node):
    def __init__(self):
        super().__init__('rotate_to_90')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Float32MultiArray, '/turtlebot_orientation', self.orientation_callback, 10)
        self.yaw = None
        self.previous_yaw = None
        self.turn_direction = 0.3  # Always turn counterclockwise (set to -0.3 for clockwise)
        self.timer = self.create_timer(0.1, self.control_loop)  # Check every 100ms

    def orientation_callback(self, msg):
        if msg.data:
            self.yaw = msg.data[0] % 360  # Keep yaw within [0, 360]
            self.previous_yaw = self.yaw  # Store last valid yaw

    def control_loop(self):
        if self.yaw is None:
            self.get_logger().info("Waiting for orientation data...")
            return

        yaw_error = (180 - self.yaw) % 360  # Compute distance to 90°
        self.get_logger().info(f"📏 Current Yaw: {self.yaw:.2f}° (Target: 90°), Error: {yaw_error:.2f}°")

        twist = Twist()
        if abs(yaw_error) > 5:  # Keep turning in the set direction
            twist.angular.z = self.turn_direction
            self.get_logger().info(f"🔄 Turning in one direction... Error: {yaw_error:.2f}°")
        else:
            self.get_logger().info("✅ Reached 90 degrees! Stopping.")
            twist.angular.z = 0.0  # Stop turning
            for _ in range(5):  # Send multiple stop commands for safety
                self.cmd_vel_pub.publish(twist)

            self.destroy_node()

        self.cmd_vel_pub.publish(twist)

def main():
    rclpy.init()
    node = RotateTo90()
    rclpy.spin(node)
    rclpy.shutdown()

