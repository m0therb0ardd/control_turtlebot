import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

class SquareMotion(Node):
    def __init__(self):
        super().__init__('square_motion')
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.current_x = None
        self.current_y = None
        self.current_theta = None
        self.start_x = None
        self.start_y = None
        self.start_theta = None

        self.state = 'waiting_for_odom'  # State machine
        self.square_size = 0.5  # Length of square sides
        self.tolerance = 0.05  # Distance tolerance for reaching waypoints
        self.current_waypoint_index = 0
        self.waypoints = []

        self.timer = self.create_timer(0.1, self.move_square)

    def odom_callback(self, msg):
        """Extracts the robot's position and orientation from the /odom topic"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, self.current_theta = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

        if self.start_x is None:  # Store the initial position once
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.start_theta = self.current_theta
            self.define_waypoints()

    def define_waypoints(self):
        """Defines waypoints relative to the start position"""
        self.waypoints = [
            (self.start_x + self.square_size, self.start_y),
            (self.start_x + self.square_size, self.start_y + self.square_size),
            (self.start_x, self.start_y + self.square_size),
            (self.start_x, self.start_y)
        ]
        self.state = 'moving_to_waypoint'
        self.get_logger().info(f'Waypoints set: {self.waypoints}')

    def move_square(self):
        """Moves the robot to each waypoint sequentially"""
        if self.state != 'moving_to_waypoint' or self.current_x is None or self.current_y is None:
            return

        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info('Square completed!')
            self.state = 'done'
            self.stop_robot()
            return

        target_x, target_y = self.waypoints[self.current_waypoint_index]

        # Compute distance to the target
        distance = math.sqrt((target_x - self.current_x)**2 + (target_y - self.current_y)**2)

        if distance < self.tolerance:
            self.get_logger().info(f'Reached waypoint {self.current_waypoint_index + 1}, rotating...')
            self.current_waypoint_index += 1
            self.rotate_90_degrees()
        else:
            self.move_forward()

    def move_forward(self):
        """Moves forward by publishing velocity commands"""
        twist = Twist()
        twist.linear.x = 0.1  # Adjust speed if needed
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def rotate_90_degrees(self):
        """Rotates the robot 90 degrees to the left"""
        twist = Twist()
        twist.angular.z = 0.5  # Rotate left
        self.cmd_vel_pub.publish(twist)

        self.get_logger().info('Rotating...')
        self.state = 'waiting_for_rotation'

        self.create_timer(2.0, self.finish_rotation)  # Wait for rotation to complete

    def finish_rotation(self):
        """Stops rotation and resumes moving to the next waypoint"""
        self.state = 'moving_to_waypoint'
        self.stop_robot()

    def stop_robot(self):
        """Stops the robot by publishing zero velocity"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = SquareMotion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
