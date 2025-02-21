

# # need to subscribe to /waypoints

# # iterate through each waypoint (waypoints publishes a /nav_msgs/Path message)

# # use /cmd_vel to convert waypoitns to velocoty commands 

# # stop when close to the waypoinf 

# # turn towards eachj waypoint beofre movign forwards

# # stop at last waypoint 



# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist, PoseStamped
# from nav_msgs.msg import Path
# import math
# from std_msgs.msg import Float32MultiArray
# import tf2_ros
# import tf_transformations
# import time
# import collections 
# import numpy as np 

# class TurtleBotWaypointFollower(Node):
#     def __init__(self):
#         super().__init__('waypoint_follower')

#         # Subscribe to waypoints
#         self.create_subscription(Path, '/waypoints', self.waypoints_callback, 10)
#         self.create_subscription(Float32MultiArray, '/turtlebot_position', self.position_callback, 10)
#         self.create_subscription(Float32MultiArray, '/turtlebot_orientation', self.orientation_callback, 10)

#         # store yaw values for smoothing
#         self.yaw_history = collections.deque(maxlen=5)  # Store last 5 yaw values for smoothing
#         self.yaw = 0.0  # Initialize yaw value


#         self.position_history = collections.deque(maxlen=5)  # Store last 5 positions for smoothing
#         self.robot_position = (0.0, 0.0)

#         # Publisher for movement
#         self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

#         self.current_waypoints = []  # Store received waypoints
#         self.current_index = 0       # Track which waypoint we are moving to
#         #self.robot_position = (0.0, 0.0)
        
#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
#         self.timer = self.create_timer(0.1, self.navigate_to_waypoint)
    
#     def update_turtlebot_position(self, x, y):
#         """Update the TurtleBot's position using the detected blue object."""
#         self.robot_position = (x, y)
#         self.get_logger().info(f"🔵 Updated TurtleBot Position: X={x:.2f}, Y={y:.2f}")

#     def orientation_callback(self, msg):
#         """Smooths TurtleBot's yaw using a moving average filter."""
#         if len(msg.data) >= 1:
#             new_yaw = msg.data[0]
            
#             # Add to history and compute moving average
#             self.yaw_history.append(new_yaw)
#             self.yaw = sum(self.yaw_history) / len(self.yaw_history)
            
#             self.get_logger().info(f"🧭 Smoothed TurtleBot Yaw: {self.yaw:.2f}°")
#         else:
#             self.get_logger().warn("⚠️ Received invalid TurtleBot orientation message!")


#     def waypoints_callback(self, msg):
#         """Receive waypoints and store them."""
#         self.current_waypoints = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
#         self.current_index = 0  # Start from the first waypoint
#         self.get_logger().info(f"✅ Received {len(self.current_waypoints)} waypoints.")

#     def position_callback(self, msg):
#         # """Update the TurtleBot's position from the YOLO node."""
#         # if len(msg.data) >= 2:
#         #     self.robot_position = (msg.data[0], msg.data[1])
#         #     self.get_logger().info(f"🔵 Updated TurtleBot Position from YOLO: X={self.robot_position[0]:.2f}, Y={self.robot_position[1]:.2f}")
#         # else:
#         #  self.get_logger().warn("⚠️ Received invalid TurtleBot position message!")
#         """Update and smooth TurtleBot's position from the YOLO node."""
#         if len(msg.data) >= 2:
#             x, y = msg.data[0], msg.data[1]

#             self.position_history.append((x, y))  # Store new position
            
#             # Compute moving average
#             avg_x = sum(p[0] for p in self.position_history) / len(self.position_history)
#             avg_y = sum(p[1] for p in self.position_history) / len(self.position_history)

#             self.robot_position = (avg_x, avg_y)
#             self.get_logger().info(f"🔵 Smoothed TurtleBot Position: X={avg_x:.2f}, Y={avg_y:.2f}")
#         else:
#             self.get_logger().warn("⚠️ Received invalid TurtleBot position message!")


#     def navigate_to_waypoint(self):
#         """Move the TurtleBot to each waypoint in sequence."""
#         if not self.current_waypoints or self.current_index >= len(self.current_waypoints):
#             self.stop_robot()
#             return
        
#         # Make sure we have a valid position update
#         if self.robot_position is None:
#             self.get_logger().warn("⚠️ Robot position not updated yet! Waiting for update...")
#             return

#         # Get current waypoint
#         goal_x, goal_y = self.current_waypoints[self.current_index]
#         robot_x, robot_y = self.robot_position  # Use real-time position from YOLO

#         # Compute distance and angle to target
#         dx = goal_x - robot_x
#         dy = goal_y - robot_y
#         distance = math.sqrt(dx**2 + dy**2)
#         angle_to_target = math.atan2(dy, dx)

#         angle_diff = math.radians(angle_to_target) - math.radians(self.yaw)  
#         angle_diff = (angle_diff + np.pi) % (2 * np.pi) - np.pi  # Normalize to [-pi, pi]
 
#         # Define movement thresholds
#         angle_threshold = 0.1  # ✅ Stop rotating when this close to target angle
#         distance_threshold = 0.1  # ✅ Stop moving when within this distance

#         # Initialize Twist message
#         twist = Twist()

#         if abs(angle_diff) > angle_threshold:  # If not facing waypoint, rotate
#             twist.angular.z = 0.5 * angle_diff  # Scale rotation
#             twist.linear.x = 0.0  # Do not move forward while rotating
#             self.get_logger().info(f"🔄 Rotating: Angle Difference={angle_diff:.3f} rad")
#         else:  # If facing waypoint, move forward
#             twist.angular.z = 0.0
#             twist.linear.x = min(0.2, distance)  # Move forward but limit speed
#             self.get_logger().info(f"🚀 Moving Forward: Distance={distance:.3f} meters")

#         # ✅ Stop at the waypoint
#         if distance < distance_threshold:
#             self.get_logger().info(f"✅ Reached waypoint {self.current_index + 1}/{len(self.current_waypoints)}")
#             self.current_index += 1
#             return

#         # Publish movement command
#         self.cmd_vel_pub.publish(twist)



#     def stop_robot(self):
#         """Stop the robot when waypoints are complete."""
#         twist = Twist()
#         self.cmd_vel_pub.publish(twist)
#         self.get_logger().info("🏁 All waypoints reached! Stopping.")

# def main():
#     rclpy.init()
#     node = TurtleBotWaypointFollower()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# ############################################################
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from std_msgs.msg import Float32MultiArray
# from nav_msgs.msg import Path
# import math

# class TurtleBotWaypointFollower(Node):
#     def __init__(self):
#         super().__init__('waypoint_follower')

#         # Subscribe to waypoints, position, and orientation
#         self.create_subscription(Path, '/waypoints', self.waypoints_callback, 10)
#         self.create_subscription(Float32MultiArray, '/turtlebot_position', self.position_callback, 10)
#         self.create_subscription(Float32MultiArray, '/turtlebot_orientation', self.orientation_callback, 10)

#         self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

#         self.current_waypoints = []
#         self.current_index = 0
#         self.robot_position = (0.0, 0.0)
#         self.yaw = None

#         self.timer = self.create_timer(0.1, self.navigate_to_waypoint)  # Run at 10Hz

#         self.reached_orientation = False  # Track if the robot has turned to face the waypoint


#     def waypoints_callback(self, msg):
#         """Receive waypoints and store them."""
#         self.current_waypoints = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
#         self.current_index = 0  # Reset to the first waypoint
#         self.get_logger().info(f"✅ Received {len(self.current_waypoints)} waypoints.")

#     def position_callback(self, msg):
#         """Update the TurtleBot's position."""
#         if len(msg.data) >= 2:
#             self.robot_position = (msg.data[0], msg.data[1])

#     def orientation_callback(self, msg):
#         """Update the TurtleBot's yaw angle."""
#         if len(msg.data) >= 1:
#             self.yaw = msg.data[0]

#     # def navigate_to_waypoint(self):
#     #     """Move the TurtleBot to the next waypoint."""
#     #     if not self.current_waypoints or self.current_index >= len(self.current_waypoints):
#     #         self.stop_robot()
#     #         return

#     #     if self.robot_position is None or self.yaw is None:
#     #         self.get_logger().warn("⚠️ Waiting for position and orientation data...")
#     #         return

#     #     # Get current waypoint
#     #     goal_x, goal_y = self.current_waypoints[self.current_index]
#     #     robot_x, robot_y = self.robot_position

#     #     # Compute angle and distance to waypoint
#     #     dx = goal_x - robot_x
#     #     dy = goal_y - robot_y
#     #     distance = math.sqrt(dx**2 + dy**2)
#     #     target_angle = math.degrees(math.atan2(dy, dx))  # Convert to degrees

#     #     # Compute target angle in 360-degree coordinates
#     #     target_angle = math.degrees(math.atan2(dy, dx))
#     #     if target_angle < 0:
#     #         target_angle += 360  # Normalize to [0, 360]

#     #     self.get_logger().info(f"📍 Target Waypoint: {goal_x:.2f}, {goal_y:.2f}")
#     #     self.get_logger().info(f"📡 Current Position: {robot_x:.2f}, {robot_y:.2f}")
#     #     self.get_logger().info(f"🧭 Current Yaw: {self.yaw:.2f}° | Target Angle: {target_angle:.2f}°")


#     #     # Ensure yaw is within [0, 360]
#     #     if self.yaw < 0:
#     #         self.yaw += 360

#     #     angle_diff = abs(target_angle - self.yaw)  # No wrapping, just direct subtraction

#     #     self.get_logger().info(f"📍 Target Waypoint: {goal_x:.2f}, {goal_y:.2f}")
#     #     self.get_logger().info(f"📡 Current Position: {robot_x:.2f}, {robot_y:.2f}")
#     #     self.get_logger().info(f"🧭 Current Yaw: {self.yaw:.2f}° | Target Angle: {target_angle:.2f}°")
#     #     self.get_logger().info(f"🔄 Angle Difference: {angle_diff:.2f}°")

#     #     twist = Twist()

#     #     if abs(angle_diff) > 5:  # If not facing waypoint, rotate counterclockwise
#     #         twist.angular.z = 0.3  # Always turn CCW, no shortest path calculation
#     #         twist.linear.x = 0.0
#     #         self.get_logger().info(f"🔄 Rotating CCW: {angle_diff:.2f}° off target")
#     #     else:  # If within threshold, move forward
#     #         twist.angular.z = 0.0
#     #         twist.linear.x = min(0.2, -(distance))  # Move but limit speed #ADDED NEGATIVE TO SEE IF IT MOVES TOWARDS WAYPOINT ISNTEAD OF AWAY
#     #         self.get_logger().info(f"🚀 Moving Forward: {distance:.3f} meters to waypoint")

#     #     if distance < 0.1:  # Reached waypoint
#     #         self.get_logger().info(f"✅ Reached waypoint {self.current_index + 1}/{len(self.current_waypoints)}")
#     #         self.current_index += 1

#     #     self.cmd_vel_pub.publish(twist)

#     # def navigate_to_waypoint(self):
#     #     """Move the TurtleBot to the next waypoint."""
#     #     if not self.current_waypoints or self.current_index >= len(self.current_waypoints):
#     #         self.stop_robot()
#     #         return

#     #     if self.robot_position is None or self.yaw is None:
#     #         self.get_logger().warn("⚠️ Waiting for position and orientation data...")
#     #         return

#     #     # Get current waypoint
#     #     goal_x, goal_y = self.current_waypoints[self.current_index]
#     #     robot_x, robot_y = self.robot_position

#     #     # Compute angle and distance to waypoint (in world frame)
#     #     dx = goal_x - robot_x
#     #     dy = goal_y - robot_y
#     #     distance = math.sqrt(dx**2 + dy**2)
#     #     target_angle = math.degrees(math.atan2(dy, dx))

#     #     # Normalize target angle to [0, 360]
#     #     if target_angle < 0:
#     #         target_angle += 360

#     #     # Normalize yaw to [0, 360]
#     #     if self.yaw < 0:
#     #         self.yaw += 360

#     #     angle_diff = abs(target_angle - self.yaw) ##SIGN OF ANGLE DIF DOESNT MATTER BC MOVIGN CCW ANYWAY

#     #     # Convert waypoint to TurtleBot frame
#     #     yaw_rad = math.radians(self.yaw)
#     #     tb_x = dx * math.cos(-yaw_rad) - dy * math.sin(-yaw_rad)
#     #     tb_y = dx * math.sin(-yaw_rad) + dy * math.cos(-yaw_rad)

#     #     # Log EVERYTHING
#     #     self.get_logger().info(f"📍 Target Waypoint (World): X={goal_x:.2f}, Y={goal_y:.2f}")
#     #     self.get_logger().info(f"📡 TurtleBot Position (World): X={robot_x:.2f}, Y={robot_y:.2f}")
#     #     self.get_logger().info(f"📍✅  Waypoint in TurtleBot Frame: X={tb_x:.2f}, Y={tb_y:.2f}")
#     #     self.get_logger().info(f"📡✅  TurtleBot Position in Its Own Frame: should be 0 0 ")
#     #     self.get_logger().info(f"🧭 Current Yaw: {self.yaw:.2f}° | Target Angle: {target_angle:.2f}°")
#     #     self.get_logger().info(f"🔄 Angle Difference: {angle_diff:.2f}°")

#     #     # Decision logic
#     #     twist = Twist()

#     #     if abs(angle_diff) > 10:  # Rotate until facing waypoint
#     #         twist.angular.z = 0.3  # Always turn CCW
#     #         twist.linear.x = 0.0
           
#     #     else:
#     #         twist.angular.z = 0.0
#     #         twist.linear.x = min(0.2, -(distance))  # Move forward but limit speed  #ADDED NEGATIVE TO SEE IF IT MOVES TOWARDS WAYPOINT ISNTEAD OF AWAY
#     #         self.get_logger().info(f"🚀 Moving Forward: {distance:.3f} meters to waypoint")

#     #     if distance < 0.1:  # Reached waypoint
#     #         self.get_logger().info(f"✅ Reached waypoint {self.current_index + 1}/{len(self.current_waypoints)}")
#     #         self.current_index += 1

#     #     self.cmd_vel_pub.publish(twist)

#     def navigate_to_waypoint(self):
#         """Move the TurtleBot to the next waypoint by first rotating, then moving."""
#         if not self.current_waypoints or self.current_index >= len(self.current_waypoints):
#             self.stop_robot()
#             return

#         if self.robot_position is None or self.yaw is None:
#             self.get_logger().warn("⚠️ Waiting for position and orientation data...")
#             return

#         # Get current waypoint
#         goal_x, goal_y = self.current_waypoints[self.current_index]
#         robot_x, robot_y = self.robot_position

#         # Compute angle and distance to waypoint
#         dx = goal_x - robot_x
#         dy = goal_y - robot_y
#         distance = math.sqrt(dx**2 + dy**2)
#         target_angle = math.degrees(math.atan2(dy, dx))

#         # Normalize angles to [0, 360]
#         if target_angle < 0:
#             target_angle += 360
#         if self.yaw < 0:
#             self.yaw += 360

#         angle_diff = target_angle - self.yaw
#         if angle_diff > 180:
#             angle_diff -= 360  # Shortest rotation direction
#         elif angle_diff < -180:
#             angle_diff += 360

#         # Log debugging info
#         self.get_logger().info(f"📍 Target Waypoint: X={goal_x:.2f}, Y={goal_y:.2f}")
#         self.get_logger().info(f"📡 TurtleBot Position: X={robot_x:.2f}, Y={robot_y:.2f}")
#         self.get_logger().info(f"🧭 Current Yaw: {self.yaw:.2f}° | Target Angle: {target_angle:.2f}°")
#         self.get_logger().info(f"🔄 Angle Difference: {angle_diff:.2f}°")

#         twist = Twist()

#         if not self.reached_orientation:
#             # Rotate first
#             if abs(angle_diff) > 5:  # Threshold to stop rotating
#                 twist.angular.z = 0.3 if angle_diff > 0 else -0.3  # Rotate in the shortest direction
#                 twist.linear.x = 0.0
#                 self.get_logger().info(f"🔄 Rotating towards waypoint: {angle_diff:.2f}° remaining")
#             else:
#                 self.reached_orientation = True  # Mark that we've completed the turn
#                 self.get_logger().info("✅ Facing the waypoint. Now moving forward.")

#         else:
#             # Move forward once orientation is correct
#             twist.angular.z = 0.0
#             twist.linear.x = min(0.2, distance)  # Move forward, limit speed
#             self.get_logger().info(f"🚀 Moving Forward: {distance:.3f} meters to waypoint")

#             if distance < 0.1:  # Reached waypoint
#                 self.get_logger().info(f"✅ Reached waypoint {self.current_index + 1}/{len(self.current_waypoints)}")
#                 self.current_index += 1
#                 self.reached_orientation = False  # Reset orientation state for the next waypoint

#         self.cmd_vel_pub.publish(twist)


#     def stop_robot(self):
#         """Stop the robot when waypoints are complete."""
#         twist = Twist()
#         self.cmd_vel_pub.publish(twist)
#         self.get_logger().info("🏁 All waypoints reached! Stopping.")

# def main():
#     rclpy.init()
#     node = TurtleBotWaypointFollower()
#     rclpy.spin(node)
#     node.destroy_node()
# #     rclpy.shutdown()


# #trying ot just send a single anlge command and then a movement command import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from std_msgs.msg import Float32MultiArray
# from nav_msgs.msg import Path
# import math
# import time
# import rclpy 
# import time 

# class TurtleBotWaypointFollower(Node):
#     def __init__(self):
#         super().__init__('waypoint_follower')

#         # Subscribe to waypoints, position, and orientation
#         self.create_subscription(Path, '/waypoints', self.waypoints_callback, 10)
#         self.create_subscription(Float32MultiArray, '/turtlebot_position', self.position_callback, 10)
#         self.create_subscription(Float32MultiArray, '/turtlebot_orientation', self.orientation_callback, 10)

#         self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

#         # State tracking
#         self.current_waypoints = []
#         self.robot_position = None
#         self.yaw = None

#     def waypoints_callback(self, msg):
#         """Receives waypoints and stores them."""
#         self.current_waypoints = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
#         self.get_logger().info(f"✅ Received {len(self.current_waypoints)} waypoints.")

#         if self.current_waypoints:
#             self.start_navigation()  # Begin movement when waypoints arrive

#     def position_callback(self, msg):
#         """Receives TurtleBot position (updates continuously for logging)."""
#         if len(msg.data) >= 2:
#             self.robot_position = (msg.data[0], msg.data[1])
#             self.get_logger().info(f"📡 Current TurtleBot Position: X={self.robot_position[0]:.3f}, Y={self.robot_position[1]:.3f}")

#     def orientation_callback(self, msg):
#         """Receives TurtleBot orientation (updates continuously for logging)."""
#         if len(msg.data) >= 1:
#             self.yaw = msg.data[0] 
#             self.get_logger().info(f"🧭 Current Yaw: {self.yaw:.2f}°")

#     def start_navigation(self):
#         """Start moving to the first waypoint using fixed rotation & movement."""
#         if not self.current_waypoints:
#             self.get_logger().warn("⚠️ No waypoints received yet!")
#             return

#         # **Wait until position and yaw are received**
#         if self.robot_position is None or self.yaw is None:
#             self.get_logger().warn("⚠️ Waiting for position & orientation before starting movement...")
#             return  # Exit and wait for the next callback
        
#         # Get the first waypoint
#         goal_x, goal_y = self.current_waypoints[0]

#         # Get the TurtleBot's current position & yaw ONCE
#         robot_x, robot_y = self.robot_position
#         current_yaw = self.yaw

#         # Compute the target angle
#         dx = goal_x - robot_x
#         dy = goal_y - robot_y
#         target_angle = math.degrees(math.atan2(dy, dx))

#         # Normalize angles to [0, 360]
#         if target_angle < 0:
#             target_angle += 360
#         if current_yaw < 0:
#             current_yaw += 360

#         # Compute shortest rotation direction
#         angle_diff = target_angle - current_yaw
#         if angle_diff > 180:
#             angle_diff -= 360
#         elif angle_diff < -180:
#             angle_diff += 360

#         # Compute the exact movement distance
#         distance = math.sqrt(dx**2 + dy**2)

#         self.get_logger().info(f"📍 Target Waypoint: X={goal_x:.2f}, Y={goal_y:.2f}")
#         self.get_logger().info(f"📡 Initial TurtleBot Position: X={robot_x:.2f}, Y={robot_y:.2f}")
#         self.get_logger().info(f"🧭 Current Yaw: {current_yaw:.2f}° | Target Angle: {target_angle:.2f}°")
#         self.get_logger().info(f"🔄 Angle Difference: {angle_diff:.2f}°")
#         self.get_logger().info(f"📏 Distance to Move: {distance:.2f} meters")

#         # **Step 1: Rotate to face the waypoint (SLOWLY)**
#         self.rotate_fixed(angle_diff)

#         # **Step 2: Move forward the exact calculated distance (SLOWLY)**
#         self.move_forward_fixed(distance)

#         # **Step 3: Stop once the movement is complete**
#         self.stop_robot()

#     def rotate_fixed(self, angle_diff):
#         """Rotates TurtleBot by the given angle difference, gradually."""
#         twist = Twist()
#         rotation_speed = 0.1  # Slower rotation speed
#         duration = abs(angle_diff) / 30.0  # Adjust time based on rotation speed

#         twist.angular.z = rotation_speed if angle_diff > 0 else -rotation_speed
#         self.cmd_vel_pub.publish(twist)

#         self.get_logger().info(f"⏳ Rotating {angle_diff:.2f}° over {duration:.2f} seconds...")
        
#         start_time = time.time()
#         while time.time() - start_time < duration:
#             self.get_logger().info(f"🔄 Still Rotating... Yaw: {self.yaw:.2f}° (Target: {angle_diff:.2f}°)")
#             time.sleep(0.5)  # Log every 0.5 seconds

#         # Stop rotation
#         self.cmd_vel_pub.publish(Twist())
#         self.get_logger().info(f"✅ Finished rotating {angle_diff:.2f}°.")

#     def move_forward_fixed(self, distance):
#         """Moves the TurtleBot forward by a fixed distance, gradually."""
#         twist = Twist()
#         move_speed = 0.1  # Slower movement speed
#         duration = distance / move_speed  # Adjust time based on speed

#         twist.linear.x = move_speed
#         self.cmd_vel_pub.publish(twist)

#         self.get_logger().info(f"🚀 Moving forward {distance:.2f}m over {duration:.2f} seconds...")
        
#         start_time = time.time()
#         while time.time() - start_time < duration:
#             self.get_logger().info(f"📡 Current Position: X={self.robot_position[0]:.3f}, Y={self.robot_position[1]:.3f} | Distance Left: {distance:.3f}")
#             time.sleep(0.5)  # Log every 0.5 seconds

#         # Stop movement
#         self.cmd_vel_pub.publish(Twist())
#         self.get_logger().info(f"🏁 Reached waypoint.")

#     def stop_robot(self):
#         """Stops the TurtleBot when waypoint is reached."""
#         twist = Twist()
#         self.cmd_vel_pub.publish(twist)
#         self.get_logger().info("🏁 Waypoint reached! Stopping.")

# def main():
#     rclpy.init()
#     node = TurtleBotWaypointFollower()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

#$#### testing basics 

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Path
import math

class TurtleBotWaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        # Subscribe to waypoints, position, and orientation
        self.create_subscription(Path, '/waypoints', self.waypoints_callback, 10)
        self.create_subscription(Float32MultiArray, '/turtlebot_position', self.position_callback, 10)
        self.create_subscription(Float32MultiArray, '/turtlebot_orientation', self.orientation_callback, 10)

        # State tracking
        self.current_waypoints = []
        self.robot_position = None
        self.yaw = None

    def waypoints_callback(self, msg):
        """Receives waypoints and stores them."""
        self.current_waypoints = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        self.get_logger().info(f"✅ Received {len(self.current_waypoints)} waypoints.")

        if self.current_waypoints:
            self.test_navigation()  # Begin logging when waypoints arrive

    def position_callback(self, msg):
        """Receives TurtleBot position."""
        if len(msg.data) >= 2:
            self.robot_position = (msg.data[0], msg.data[1])
            self.get_logger().info(f"📡 Current TurtleBot Position: X={self.robot_position[0]:.3f}, Y={self.robot_position[1]:.3f}")

    def orientation_callback(self, msg):
        """Receives TurtleBot orientation."""
        if len(msg.data) >= 1:
            self.yaw = msg.data[0] 
            self.get_logger().info(f"🧭 Current Yaw: {self.yaw:.2f}°")

    def test_navigation(self):
        """Logs all movement calculations but DOES NOT move the TurtleBot."""
        if not self.current_waypoints:
            self.get_logger().warn("⚠️ No waypoints received yet!")
            return

        if self.robot_position is None or self.yaw is None:
            self.get_logger().warn("⚠️ Waiting for position & orientation before testing...")
            return  # Exit and wait for the next callback
        
        # Get the first waypoint
        goal_x, goal_y = self.current_waypoints[0]

        # Get the TurtleBot's current position & yaw
        robot_x, robot_y = self.robot_position
        current_yaw = self.yaw

        # Compute the target angle
        dx = goal_x - robot_x
        dy = goal_y - robot_y
        target_angle = math.degrees(math.atan2(dy, dx))

        # Normalize angles to [0, 360]
        if target_angle < 0:
            target_angle += 360
        if current_yaw < 0:
            current_yaw += 360

        # Compute shortest rotation direction
        angle_diff = -( target_angle - current_yaw ) ## orientaion thing 
        if angle_diff > 180:
            angle_diff -= 360
        elif angle_diff < -180:
            angle_diff += 360

        # Compute movement distance
        distance = math.sqrt(dx**2 + dy**2)

        # 🚨 **LOG EVERYTHING**
        self.get_logger().info(f"📍 Target Waypoint: X={goal_x:.2f}, Y={goal_y:.2f}")
        self.get_logger().info(f"📡 TurtleBot Initial Position: X={robot_x:.2f}, Y={robot_y:.2f}")
        self.get_logger().info(f"🧭 Current Yaw: {current_yaw:.2f}° | Target Angle: {target_angle:.2f}°")
        self.get_logger().info(f"🔄 Angle Difference: {angle_diff:.2f}°")
        self.get_logger().info(f"📏 Distance to Move: {distance:.2f} meters")

        self.get_logger().info(f"✅ Done logging! Check these values before enabling movement.")

def main():
    rclpy.init()
    node = TurtleBotWaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
