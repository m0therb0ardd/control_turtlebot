# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# import time

# class SquareMover(Node):
#     def __init__(self):
#         super().__init__('square_mover')
#         self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.timer = self.create_timer(2.0, self.run_square)  # Run every 2 seconds
#         self.step = 0  # Keep track of movement steps

#     def run_square(self):
#         """Moves the TurtleBot in a small square (5-inch sides)."""
#         twist = Twist()
        
#         if self.step % 2 == 0:  # Move Forward Step
#             self.get_logger().info(f"🚀 Moving Forward Step {self.step//2 + 1}/4")
#             twist.linear.x = 0.05  # Move forward at 0.05 m/s
#             self.cmd_vel_pub.publish(twist)
#             time.sleep(2.54)  # Move forward for ~5 inches (0.127m / 0.05m/s)

#         else:  # Rotate Step
#             self.get_logger().info(f"🔄 Rotating 90° CCW")
#             twist.angular.z = 0.5  # Rotate at 0.5 rad/s
#             self.cmd_vel_pub.publish(twist)
#             time.sleep(3.14)  # Rotate for ~90° (π/2 radians at 0.5 rad/s)

#         self.cmd_vel_pub.publish(Twist())  # Stop movement
#         self.step += 1

#         if self.step >= 8:  # 4 moves + 4 turns = 8 steps
#             self.get_logger().info("🏁 Square Complete! Stopping.")
#             self.destroy_node()  # Stop the node

# def main():
#     rclpy.init()
#     node = SquareMover()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from std_msgs.msg import Float32MultiArray
# import math
# import time

# class SquareMover(Node):
#     def __init__(self):
#         super().__init__('square_mover')

#         self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.create_subscription(Float32MultiArray, '/turtlebot_position', self.position_callback, 10)
#         self.create_subscription(Float32MultiArray, '/turtlebot_orientation', self.orientation_callback, 10)

#         self.robot_position = None
#         self.yaw = None
#         self.square_waypoints = []
#         self.current_index = 0
#         self.ready = False

#     def position_callback(self, msg):
#         """Receives initial position & sets waypoints."""
#         if self.robot_position is None:
#             self.robot_position = (msg.data[0], msg.data[1])
#             self.get_logger().info(f"📡 Initial Position: X={self.robot_position[0]:.3f}, Y={self.robot_position[1]:.3f}")

#             # Define waypoints in a 5-inch square (~0.127m per side)
#             x, y = self.robot_position
#             self.square_waypoints = [
#                 (x + 0.127, y),  # Move Right
#                 (x + 0.127, y + 0.127),  # Move Up
#                 (x, y + 0.127),  # Move Left
#                 (x, y)  # Move Down (Back to Start)
#             ]

#             self.get_logger().info(f"📍 Set Square Waypoints: {self.square_waypoints}")

#     def orientation_callback(self, msg):
#         """Receives initial orientation."""
#         if self.yaw is None:
#             self.yaw = msg.data[0]
#             self.get_logger().info(f"🧭 Initial Yaw: {self.yaw:.2f}°")

#             if self.robot_position:
#                 self.ready = True  # Start when both position & yaw are received
#                 self.move_to_next_waypoint()

#     def move_to_next_waypoint(self):
#         """Moves to the next waypoint using fixed rotation & movement."""
#         if self.current_index >= len(self.square_waypoints):
#             self.get_logger().info("🏁 Square Complete! Stopping.")
#             self.stop_robot()
#             return

#         goal_x, goal_y = self.square_waypoints[self.current_index]
#         robot_x, robot_y = self.robot_position
#         current_yaw = self.yaw

#         # Compute target angle
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

#         # Compute exact movement distance
#         distance = math.sqrt(dx**2 + dy**2)

#         self.get_logger().info(f"📍 Target: X={goal_x:.3f}, Y={goal_y:.3f}")
#         self.get_logger().info(f"🧭 Current Yaw: {current_yaw:.2f}° | Target Angle: {target_angle:.2f}°")
#         self.get_logger().info(f"🔄 Rotating: {angle_diff:.2f}° | 📏 Moving: {distance:.3f}m")

#         # **Step 1: Rotate**
#         self.rotate_fixed(angle_diff)

#         # **Step 2: Move Forward**
#         self.move_forward_fixed(distance)

#         # **Step 3: Update Position & Move to Next Waypoint**
#         self.robot_position = (goal_x, goal_y)
#         self.yaw = target_angle  # Assume perfect rotation
#         self.current_index += 1

#         # Move to next waypoint after delay
#         time.sleep(1.0)
#         self.move_to_next_waypoint()

#     def rotate_fixed(self, angle_diff):
#         """Rotates a fixed angle without checking yaw updates."""
#         self.get_logger().info(f"🔄 Rotate Fixed Called! Angle: {angle_diff:.2f}°")
#         twist = Twist()
#         twist.angular.z = 0.5 if angle_diff > 0 else -0.5
#         self.cmd_vel_pub.publish(twist)
#         self.get_logger().info(f"🔄 Rotating {angle_diff:.2f}° at {twist.angular.z} rad/s")



#     def move_forward_fixed(self, distance):
#         """Moves a fixed distance forward."""
#         self.get_logger().info(f"🚀 Move Forward Fixed Called! Distance: {distance:.2f}m")
#         twist = Twist()
#         twist.linear.x = 0.05
#         self.cmd_vel_pub.publish(twist)
#         time.sleep(distance / 0.05)  # Move at 0.05 m/s
#         self.cmd_vel_pub.publish(Twist())  # Stop movement

#     def stop_robot(self):
#         """Stops the TurtleBot."""
#         self.cmd_vel_pub.publish(Twist())
#         self.get_logger().info("🏁 Movement Complete!")

# def main():
#     rclpy.init()
#     node = SquareMover()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()






import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import math
import time

class SquareMover(Node):
    def __init__(self):
        super().__init__('square_mover')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Float32MultiArray, '/turtlebot_position', self.position_callback, 10)
        self.create_subscription(Float32MultiArray, '/turtlebot_orientation', self.orientation_callback, 10)

        self.robot_position = None
        self.yaw = None
        self.square_waypoints = []
        self.current_index = 0
        self.ready = False

    def position_callback(self, msg):
        """Receives initial position & sets waypoints."""
        if self.robot_position is None:
            self.robot_position = (msg.data[0], msg.data[1])
            self.get_logger().info(f"📡 Initial Position: X={self.robot_position[0]:.3f}, Y={self.robot_position[1]:.3f}")

            # Define waypoints in a 5-inch square (~0.127m per side)
            x, y = self.robot_position
            self.square_waypoints = [
                (x + 0.127, y),  # rightttt
                (x + 0.127, y + 0.127),  # upppp
                (x, y + 0.127),  # leftttt 
                (x, y)  # downnn (back to start)
            ]

            self.get_logger().info(f"📍 Set Square Waypoints: {self.square_waypoints}")

    # def orientation_callback(self, msg):
    #     """Receives initial orientation."""
    #     if self.yaw is None:
    #         self.yaw = msg.data[0]
    #         self.get_logger().info(f"🧭 Initial Yaw: {self.yaw:.2f}°")

    #         if self.robot_position:
    #             self.ready = True  # Start when both position & yaw are received
    #             self.move_to_next_waypoint()

    def orientation_callback(self, msg):
        """Receives yaw directly from YOLO node."""
        if len(msg.data) >= 1:  # Ensure there's data
            self.yaw = msg.data[0]  # Store received yaw
            self.get_logger().info(f"🧭 Received TurtleBot Yaw from YOLO: {self.yaw:.2f}°")

            if self.robot_position:  # If we also have position, start moving!
                self.ready = True
                self.move_to_next_waypoint()


    def move_to_next_waypoint(self):
        """Moves to the next waypoint using fixed rotation & movement in TurtleBot's local frame."""
        if self.current_index >= len(self.square_waypoints):
            #self.get_logger().info("🏁 Square Complete! Stopping.")
            self.stop_robot()
            return

        # Get global waypoint
        goal_x_world, goal_y_world = self.square_waypoints[self.current_index]

        # Get current position & yaw
        robot_x, robot_y = self.robot_position
        current_yaw = math.radians(self.yaw)  # Convert to radians for math functions

        # I DOTN TRUST THIS !!!!!!!!!!!!!!!!!!!!!!!!!!! FROM HERE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        # Convert world waypoint to TurtleBot’s frame
        dx_world = goal_x_world - robot_x
        dy_world = goal_y_world - robot_y

        # Transform to TurtleBot’s local frame (rotating by -current_yaw)
        goal_x_local = dx_world * math.cos(-current_yaw) - dy_world * math.sin(-current_yaw)
        goal_y_local = dx_world * math.sin(-current_yaw) + dy_world * math.cos(-current_yaw)

        # Compute the target angle in TurtleBot’s frame
        target_angle = math.degrees(math.atan2(goal_y_local, goal_x_local))

        # Compute the shortest rotation direction
        angle_diff = target_angle  # Since we are already in the TurtleBot frame
        if angle_diff > 180:
            angle_diff -= 360
        elif angle_diff < -180:
            angle_diff += 360

        # I DONT TRUST THIS TO HERE  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!11!!!!!!!!!!!!!!!!!!!!!!!!!!!

        # Compute movement distance (TurtleBot frame)
        distance = math.sqrt(goal_x_local**2 + goal_y_local**2)

        self.get_logger().info(f"📍 Target (Local Turtlebot Frame): X={goal_x_local:.3f}, Y={goal_y_local:.3f}")
        self.get_logger().info(f"🧭 Current Yaw: {self.yaw:.2f}° | Target Angle (Local): {target_angle:.2f}°")
        self.get_logger().info(f"🔄 Rotating: {angle_diff:.2f}° | 📏 Moving: {distance:.3f}m")

        # **Step 1: Rotate**
        self.rotate_fixed(angle_diff, lambda: self.move_forward_fixed(distance))



    def rotate_fixed(self, angle_diff, callback):
        """Rotates a fixed angle without checking yaw updates, then calls the next step."""
        self.get_logger().info(f"🔄 Rotate Fixed Called! Angle: {angle_diff:.2f}°")
        twist = Twist()
        twist.angular.z = 0.5 if angle_diff > 0 else -0.5
        self.cmd_vel_pub.publish(twist)

        # Compute rotation time
        time_to_rotate = abs(angle_diff) / 60  # Assuming ~60°/sec speed
        #self.get_logger().info(f"⏳ Rotating for {time_to_rotate:.2f} seconds...")

        # Delay then move forward
        self.create_timer(time_to_rotate, lambda: self.stop_rotation(callback))

    def stop_rotation(self, callback):
        """Stops rotation and proceeds to move forward."""
        self.cmd_vel_pub.publish(Twist())  # Stop rotation
        #self.get_logger().info(f"✅ Rotation complete! Now moving forward...")
        callback()  # Proceed to forward movement

    def move_forward_fixed(self, distance):
        """Moves a fixed distance forward and waits before moving to next waypoint."""
       # self.get_logger().info(f"🚀 Move Forward Fixed Called! Distance: {distance:.2f}m")
        twist = Twist()
        twist.linear.x = 0.1
        self.cmd_vel_pub.publish(twist)

        # Compute movement time
        time_to_move = distance / 0.1  # Speed is 0.1 m/s
        #self.get_logger().info(f"⏳ Moving forward for {time_to_move:.2f} seconds...")

        # Delay then move to next waypoint
        self.create_timer(time_to_move, lambda: self.stop_movement())

    def stop_movement(self):
        """Stops the TurtleBot after moving forward."""
        self.cmd_vel_pub.publish(Twist())  # Stop movement
        #self.get_logger().info("✅ Forward movement complete!")

        # Move to the next waypoint
        self.current_index += 1
        self.move_to_next_waypoint()

    def stop_robot(self):
        """Stops the TurtleBot."""
        self.cmd_vel_pub.publish(Twist())
        #self.get_logger().info("🏁 Movement Complete!")

def main():
    rclpy.init()
    node = SquareMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



