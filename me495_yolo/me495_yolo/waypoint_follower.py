

# need to subscribe to /waypoints

# iterate through each waypoint (waypoints publishes a /nav_msgs/Path message)

# use /cmd_vel to convert waypoitns to velocoty commands 

# stop when close to the waypoinf 

# turn towards eachj waypoint beofre movign forwards

# stop at last waypoint 



import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
import math
from std_msgs.msg import Float32MultiArray
import tf2_ros
import tf_transformations
import time
import collections 

class TurtleBotWaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        # Subscribe to waypoints
        self.subscription = self.create_subscription(
            Path,
            '/waypoints',
            self.waypoints_callback,
            10
        )
        self.create_subscription(Float32MultiArray, '/turtlebot_position', self.position_callback, 10)

        self.position_history = collections.deque(maxlen=5)  # Store last 5 positions for smoothing
        self.robot_position = (0.0, 0.0)


        # Publisher for movement
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.current_waypoints = []  # Store received waypoints
        self.current_index = 0       # Track which waypoint we are moving to
        self.robot_position = (0.0, 0.0)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.yaw = 0.0  # Initialize yaw
        self.create_timer(0.1, self.update_yaw)  # Update yaw every 100ms

    

        self.timer = self.create_timer(0.1, self.navigate_to_waypoint)
    
    def update_turtlebot_position(self, x, y):
        """Update the TurtleBot's position using the detected blue object."""
        self.robot_position = (x, y)
        self.get_logger().info(f"🔵 Updated TurtleBot Position: X={x:.2f}, Y={y:.2f}")

    # def update_yaw(self):
        # try:
        #     # yaw relative to cameras detected turtlebot position 
        #     transform = self.tf_buffer.lookup_transform("turtlebot_blue_object", "turtlebot_base", rclpy.time.Time())
        #     quat = transform.transform.rotation
        #     _, _, self.yaw = tf_transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        #     self.get_logger().info(f"🔄 Updated yaw: {self.yaw:.2f} rad")
        # except Exception as e:
        #     self.get_logger().warn(f"⚠️ Could not update yaw: {e}")
    # def update_yaw(self):
    #     try:
    #         transform = self.tf_buffer.lookup_transform("odom", "base_link", rclpy.time.Time())
    #         quat = transform.transform.rotation
    #         _, _, self.yaw = tf_transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
    #         self.get_logger().info(f"🔄 Updated yaw from /odom: {self.yaw:.2f} rad")
    #     except Exception as e:
    #         self.get_logger().warn(f"⚠️ Could not update yaw from /odom: {e}")

    def update_yaw(self):
        try:
            transform = self.tf_buffer.lookup_transform("odom", "turtlebot_base", rclpy.time.Time())
            quat = transform.transform.rotation
            _, _, self.yaw = tf_transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
            self.get_logger().info(f"🔄 Updated yaw: {self.yaw:.2f} rad")
        except Exception as e:
            self.get_logger().warn(f"⚠️ Could not update yaw from /odom: {e}")




    def waypoints_callback(self, msg):
        """Receive waypoints and store them."""
        self.current_waypoints = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        self.current_index = 0  # Start from the first waypoint
        self.get_logger().info(f"✅ Received {len(self.current_waypoints)} waypoints.")

    def position_callback(self, msg):
        # """Update the TurtleBot's position from the YOLO node."""
        # if len(msg.data) >= 2:
        #     self.robot_position = (msg.data[0], msg.data[1])
        #     self.get_logger().info(f"🔵 Updated TurtleBot Position from YOLO: X={self.robot_position[0]:.2f}, Y={self.robot_position[1]:.2f}")
        # else:
        #  self.get_logger().warn("⚠️ Received invalid TurtleBot position message!")
        """Update and smooth TurtleBot's position from the YOLO node."""
        if len(msg.data) >= 2:
            x, y = msg.data[0], msg.data[1]

            self.position_history.append((x, y))  # Store new position
            
            # Compute moving average
            avg_x = sum(p[0] for p in self.position_history) / len(self.position_history)
            avg_y = sum(p[1] for p in self.position_history) / len(self.position_history)

            self.robot_position = (avg_x, avg_y)
            self.get_logger().info(f"🔵 Smoothed TurtleBot Position: X={avg_x:.2f}, Y={avg_y:.2f}")
        else:
            self.get_logger().warn("⚠️ Received invalid TurtleBot position message!")



    # def navigate_to_waypoint(self):
    #     """Move the TurtleBot to each waypoint in sequence."""
    #     if not self.current_waypoints or self.current_index >= len(self.current_waypoints):
    #         self.stop_robot()
    #         return
        
    #     # Prevent error if robot_position is None --> if delay in detection 
    #     if self.robot_position is None:
    #         self.get_logger().warn("⚠️ Robot position not updated yet! Waiting for update...")
    #         return

    #     goal_x, goal_y = self.current_waypoints[self.current_index]
    #     robot_x, robot_y = self.robot_position  

    #     distance = math.sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2)
    #     angle_to_target = math.atan2(goal_y - robot_y, goal_x - robot_x)

    #     if distance < 0.1:  # Close enough to waypoint
    #         self.get_logger().info(f"✅ Reached waypoint {self.current_index + 1}/{len(self.current_waypoints)}")
    #         self.current_index += 1
    #         return

    #     # Create movement command
    #     twist = Twist()
    #     twist.linear.x = min(0.2, distance)  # Move forward
    #     twist.angular.z = min(0.5, angle_to_target)  # Turn towards waypoint

    #     self.cmd_vel_pub.publish(twist)

    # def navigate_to_waypoint(self):
    #     """Move the TurtleBot to each waypoint in sequence."""
    #     if not self.current_waypoints or self.current_index >= len(self.current_waypoints):
    #         self.stop_robot()
    #         return
        
    #     # Make sure we have a valid position update
    #     if self.robot_position is None:
    #         self.get_logger().warn("⚠️ Robot position not updated yet! Waiting for update...")
    #         return

    #     # Get current waypoint
    #     goal_x, goal_y = self.current_waypoints[self.current_index]
    #     robot_x, robot_y = self.robot_position  # Use real-time position from YOLO

    #     # # Compute distance and angle to target
    #     # distance = math.sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2)
    #     # angle_to_target = math.atan2(goal_y - robot_y, goal_x - robot_x)

    #     # # Print debug info
    #     # self.get_logger().info(f"🎯 Target: X={goal_x:.2f}, Y={goal_y:.2f}")
    #     # self.get_logger().info(f"🤖 TurtleBot Position: X={robot_x:.2f}, Y={robot_y:.2f}")
    #     # self.get_logger().info(f"📏 Distance to Waypoint: {distance:.3f} meters")

    #     # # If close enough to waypoint, move to the next one
    #     # if distance < 0.1:
    #     #     self.get_logger().info(f"✅ Reached waypoint {self.current_index + 1}/{len(self.current_waypoints)}")
    #     #     self.current_index += 1
    #     #     return

    #     # # Create movement command
    #     # twist = Twist()

    #     # # First, rotate toward the waypoint
    #     # angle_diff = angle_to_target  # Simplified for now; could be improved with real orientation
    #     # # if abs(angle_diff) > 0.1:  
    #     # #     twist.angular.z = 0.5 if angle_diff > 0 else -0.5  # Turn toward target
        
    #     # angle_threshold = 0.2  # Small threshold for when to stop turning

    #     # if abs(angle_to_target - self.yaw) > angle_threshold:
    #     #     twist.angular.z = 0.3 * (angle_to_target - self.yaw)  # Scales rotation speed
    #     #     twist.linear.x = 0.0  # Ensure it doesn't move while turning
    #     # else:
    #     #     twist.angular.z = 0.0  # Stop turning when aligned
    #     #     twist.linear.x = 0.2  # Start moving forward


    #     # #pushkars idea use cobtrol from homework 1 
    

    #     # # Debug message before publishing command
    #     # self.get_logger().info(f"🚀 Sending /cmd_vel: Linear={twist.linear.x:.2f}, Angular={twist.angular.z:.2f}")

    #     # # Publish movement command
    #     # self.cmd_vel_pub.publish(twist)
    #     velocity_msg = Twist() #twist is a message type in ROS to describe velocities in linear and angular components
    #     #first find distance btwn turtle and the waypoint broken into x and y component
    #     dx = goal_x - robot_x
    #     dy = goal_y - robot_y
    #     distance = math.sqrt(dx**2+dy**2)


    #     velocity_msg.linear.x = min(0.50, distance)
    #     angle_to_target = math.atan2(dy, dx)
    #     velocity_msg.angular.z = angle_to_target- self.current_orientation
    #     self.velocity_publisher.publish(velocity_msg)
    # #     self.get_logger().info

    # def navigate_to_waypoint(self):
    #     """Move the TurtleBot to each waypoint in sequence using /odom yaw."""
    #     if not self.current_waypoints or self.current_index >= len(self.current_waypoints):
    #         self.stop_robot()
    #         return
        
    #     if self.robot_position is None:
    #         self.get_logger().warn("⚠️ Robot position not updated yet! Waiting for update...")
    #         return

    #     goal_x, goal_y = self.current_waypoints[self.current_index]
    #     robot_x, robot_y = self.robot_position  

    #     dx = goal_x - robot_x
    #     dy = goal_y - robot_y
    #     distance = math.sqrt(dx**2 + dy**2)
    #     angle_to_target = math.atan2(dy, dx)

    #     # Compute yaw error
    #     yaw_error = angle_to_target - self.yaw
    #     yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))  # Normalize to [-pi, pi]

    #     twist = Twist()

    #     # **Step 1: Rotate towards the waypoint**
    #     if abs(yaw_error) > 0.2:  # If yaw error is significant
    #         twist.angular.z = 0.5 * yaw_error  # Proportional control
    #         twist.linear.x = 0.0  # Don't move forward yet
    #     else:
    #         # **Step 2: Move forward once aligned**
    #         twist.linear.x = min(0.2, distance)  # Limit speed
    #         twist.angular.z = 0.0  # Stop turning

    #     self.get_logger().info(f"🚀 Sending /cmd_vel: Linear={twist.linear.x:.2f}, Angular={twist.angular.z:.2f}")
    #     self.cmd_vel_pub.publish(twist)

    #     # **Step 3: Check if we reached the waypoint**
    #     if distance < 0.1:
    #         self.get_logger().info(f"✅ Reached waypoint {self.current_index + 1}/{len(self.current_waypoints)}")
    #         self.current_index += 1

    def navigate_to_waypoint(self):
        """Move the TurtleBot to each waypoint in sequence."""
        if not self.current_waypoints or self.current_index >= len(self.current_waypoints):
            self.stop_robot()
            return
        
        # Make sure we have a valid position update
        if self.robot_position is None:
            self.get_logger().warn("⚠️ Robot position not updated yet! Waiting for update...")
            return

        # Get current waypoint
        goal_x, goal_y = self.current_waypoints[self.current_index]
        robot_x, robot_y = self.robot_position  # Use real-time position from YOLO

        # Compute distance and angle to target
        dx = goal_x - robot_x
        dy = goal_y - robot_y
        distance = math.sqrt(dx**2 + dy**2)
        angle_to_target = math.atan2(dy, dx)

        # Get yaw (current orientation from /odom)
        angle_diff = angle_to_target - self.yaw  # Difference between current yaw and target
        angle_diff = (angle_diff + np.pi) % (2 * np.pi) - np.pi  # Normalize to [-pi, pi]

        # Define movement thresholds
        angle_threshold = 0.1  # ✅ Stop rotating when this close to target angle
        distance_threshold = 0.1  # ✅ Stop moving when within this distance

        # Initialize Twist message
        twist = Twist()

        if abs(angle_diff) > angle_threshold:  # If not facing waypoint, rotate
            twist.angular.z = 0.5 * angle_diff  # Scale rotation
            twist.linear.x = 0.0  # Do not move forward while rotating
            self.get_logger().info(f"🔄 Rotating: Angle Difference={angle_diff:.3f} rad")
        else:  # If facing waypoint, move forward
            twist.angular.z = 0.0
            twist.linear.x = min(0.2, distance)  # Move forward but limit speed
            self.get_logger().info(f"🚀 Moving Forward: Distance={distance:.3f} meters")

        # ✅ Stop at the waypoint
        if distance < distance_threshold:
            self.get_logger().info(f"✅ Reached waypoint {self.current_index + 1}/{len(self.current_waypoints)}")
            self.current_index += 1
            return

        # Publish movement command
        self.cmd_vel_pub.publish(twist)



    def stop_robot(self):
        """Stop the robot when waypoints are complete."""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("🏁 All waypoints reached! Stopping.")

def main():
    rclpy.init()
    node = TurtleBotWaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
