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


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import math
import time
import tf2_ros
import tf_transformations
from geometry_msgs.msg import TransformStamped
import numpy as np

class SquareMover(Node):
    def __init__(self):
        super().__init__('square_mover')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Float32MultiArray, '/turtlebot_position_april', self.position_callback, 10)
        self.create_subscription(Float32MultiArray, '/turtlebot_orientation_april', self.orientation_callback, 10)

        self.robot_position = None
        self.yaw = None
        self.square_waypoints = []
        self.current_index = 0
        self.ready = False

        # for waypoint tranformation 
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)


    def position_callback(self, msg):
        """Receives initial position & sets waypoints."""
        if self.robot_position is None:
            self.robot_position = (msg.data[0], msg.data[1])
            self.get_logger().info(f"📡 Initial Position: X={self.robot_position[0]:.3f}, Y={self.robot_position[1]:.3f}")

            # ✅ Define waypoints properly
            x, y = self.robot_position
            waypoints_world = [
                (x + 0.127, y, 0.0),  # right
                (x + 0.127, y + 0.127, 0.0),  # up
                (x, y + 0.127, 0.0),  # left
                (x, y, 0.0)  # down (back to start)
            ]

            self.get_logger().info(f"📍 Original Waypoints (World Frame): {waypoints_world}")

            # ✅ Transform waypoints and store them
            self.square_waypoints = self.transform_waypoints_to_turtlebot(waypoints_world)

            if self.square_waypoints:  # Ensure waypoints were successfully transformed
                self.get_logger().info(f"📍 Transformed Waypoints (TurtleBot Frame): {self.square_waypoints}")
            else:
                self.get_logger().error("🚨 Waypoints transformation failed! No transformed waypoints available.")

            # ✅ Check if both position and orientation are available
            if self.yaw is not None and self.square_waypoints:
                self.ready = True
                self.move_to_next_waypoint()


    def orientation_callback(self, msg):
        """Receives yaw directly from YOLO node."""
        if len(msg.data) >= 1:  # Ensure there's data
            self.yaw = msg.data[0]  # Store received yaw
            self.get_logger().info(f"🧭 Received TurtleBot Yaw from YOLO: {self.yaw:.2f}°")

            # ✅ Check if both position and waypoints are available
            if self.robot_position is not None and self.square_waypoints:
                self.ready = True
                self.move_to_next_waypoint()


    
    # def transform_waypoints_to_turtlebot(self, waypoints):
    #     """Transforms waypoints from the camera frame to the TurtleBot’s local frame using TF lookups."""
    #     transformed_waypoints = []

    #     for x, y, z in waypoints:
    #         try:
    #             # Ensure TF buffer is initialized
    #             # if not hasattr(self, "tf_buffer") or self.tf_buffer is None:
    #             #     self.get_logger().error("❌ TF buffer not initialized!")
    #             #     return []

    #             # Wait for transform to become available
    #             timeout_sec = 5.0  # Max wait time
    #             start_time = self.get_clock().now().seconds_nanoseconds()[0]

    #             while not self.tf_buffer.lookup_transform("turtlebot_position_april", "camera_color_optical_frame", rclpy.time.Time()):
    #                 if self.get_clock().now().seconds_nanoseconds()[0] - start_time > timeout_sec:
    #                     self.get_logger().error("🚨 Transform still NOT available after waiting!")
    #                     return []
    #                 self.get_logger().warn("⏳ Waiting for transform from camera → turtlebot_position_april...")
    #                 time.sleep(0.5)  # Small delay before retrying


    #             # Check if the transform is available BEFORE looking it up
    #             if not self.tf_buffer.lookup_transform("turtlebot_position_april", "camera_color_optical_frame", rclpy.time.Time()):
    #                 self.get_logger().error("🚨 Transform from camera → turtlebot_position_april is NOT available!")
    #                 return []

    #             # Lookup the transform from the camera frame to the TurtleBot frame
    #             transform =self.tf_buffer.lookup_transform("turtlebot_position_april", "camera_color_optical_frame", rclpy.time.Time())

    #             # Extract translation & rotation
    #             trans = transform.transform.translation
    #             rot = transform.transform.rotation

    #             # Convert quaternion to a rotation matrix
    #             quat = [rot.x, rot.y, rot.z, rot.w]
    #             rot_matrix = tf_transformations.quaternion_matrix(quat)

    #             # Convert waypoint to a homogeneous coordinate (4x1 vector)
    #             point = np.array([x, y, z, 1]).reshape(4, 1)

    #             # Apply transformation (Rotation + Translation)
    #             transformed_point = np.dot(rot_matrix, point)
    #             transformed_x = transformed_point[0, 0] + trans.x
    #             transformed_y = transformed_point[1, 0] + trans.y
    #             transformed_z = transformed_point[2, 0] + trans.z

    #             self.get_logger().info(f"✅ Transformed: X={transformed_x:.3f}, Y={transformed_y:.3f}, Z={transformed_z:.3f}")

    #             transformed_waypoints.append((transformed_x, transformed_y, transformed_z))

    #         except Exception as e:
    #             self.get_logger().error(f"🚨 Failed to transform waypoint: {e}")

    #     return transformed_waypoints

    def transform_waypoints_to_turtlebot(self, waypoints):
        """Transforms waypoints from the camera frame to the TurtleBot’s local frame using TF lookups."""
        transformed_waypoints = []

        for x, y, z in waypoints:
            try:
                timeout_sec = 20.0  # More time for transform to become available
                start_time = self.get_clock().now().seconds_nanoseconds()[0]

                # ✅ Print all available TF frames for debugging
                frames = self.tf_buffer.all_frames_as_yaml()
                self.get_logger().info(f"📡 Available TF Frames: \n{frames}")

                # ✅ Correct the frame order (turtlebot_position_april → camera_color_optical_frame)
                while not self.tf_buffer.can_transform("tag36h11:4", "camera_color_optical_frame", rclpy.time.Time()):
                    if self.get_clock().now().seconds_nanoseconds()[0] - start_time > timeout_sec:
                        self.get_logger().error("🚨 Transform still NOT available after waiting!")
                        return []
                    self.get_logger().warn("⏳ Waiting for transform from camera_color_optical_frame →  turtleturtlebot_position_april...")
                    time.sleep(0.5)  # Small delay before retrying

                self.get_logger().warn("FOUNDDDDDDDDDDDDDDDDDDDDDDD")   
                # ✅ Correct the transform lookup command
                transform = self.tf_buffer.lookup_transform("tag36h11:4", "camera_color_optical_frame", rclpy.time.Time())

                # ✅ Extract translation & rotation
                trans = transform.transform.translation
                rot = transform.transform.rotation

                # ✅ Convert quaternion to a rotation matrix
                quat = [rot.x, rot.y, rot.z, rot.w]
                rot_matrix = tf_transformations.quaternion_matrix(quat)

                # ✅ Convert waypoint to a homogeneous coordinate (4x1 vector)
                point = np.array([x, y, z, 1]).reshape(4, 1)

                # ✅ Apply transformation (Rotation + Translation)
                transformed_point = np.dot(rot_matrix, point)
                transformed_x = transformed_point[0, 0] + trans.x
                transformed_y = transformed_point[1, 0] + trans.y
                transformed_z = transformed_point[2, 0] + trans.z

                self.get_logger().info(f"✅ Transformed: X={transformed_x:.3f}, Y={transformed_y:.3f}, Z={transformed_z:.3f}")

                transformed_waypoints.append((transformed_x, transformed_y, transformed_z))

            except Exception as e:
                self.get_logger().error(f"🚨 Failed to transform waypoint: {e}")

        return transformed_waypoints


    # def move_to_next_waypoint(self):
    #     """Moves to the next waypoint using fixed rotation & movement in TurtleBot's local frame."""
    #     if self.current_index >= len(self.square_waypoints):
    #         #self.get_logger().info("🏁 Square Complete! Stopping.")
    #         self.stop_robot()
    #         return

    #     # Get global waypoint
    #     goal_x_world, goal_y_world = self.square_waypoints[self.current_index][:2]  # Ignore Z

    #     # Get current position & yaw
    #     robot_x, robot_y = self.robot_position
    #     current_yaw = math.radians(self.yaw)  # Convert to radians for math functions

    #     # I DOTN TRUST THIS !!!!!!!!!!!!!!!!!!!!!!!!!!! FROM HERE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    #     # Convert world waypoint to TurtleBot’s frame
    #     dx_world = goal_x_world - robot_x
    #     dy_world = goal_y_world - robot_y

    #     # Transform to TurtleBot’s local frame (rotating by -current_yaw)
    #     goal_x_local = dx_world * math.cos(-current_yaw) - dy_world * math.sin(-current_yaw)
    #     goal_y_local = dx_world * math.sin(-current_yaw) + dy_world * math.cos(-current_yaw)

    #     # Compute the target angle in TurtleBot’s frame
    #     target_angle = math.degrees(math.atan2(goal_y_local, goal_x_local))

    #     # Compute the shortest rotation direction
    #     angle_diff = target_angle  # Since we are already in the TurtleBot frame
    #     if angle_diff > 180:
    #         angle_diff -= 360
    #     elif angle_diff < -180:
    #         angle_diff += 360

    #     # I DONT TRUST THIS TO HERE  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!11!!!!!!!!!!!!!!!!!!!!!!!!!!!

    #     # Compute movement distance (TurtleBot frame)
    #     distance = math.sqrt(goal_x_local**2 + goal_y_local**2)

    #     self.get_logger().info(f"📍 Target (Local Turtlebot Frame): X={goal_x_local:.3f}, Y={goal_y_local:.3f}")
    #     self.get_logger().info(f"🧭 Current Yaw: {self.yaw:.2f}° | Target Angle (Local): {target_angle:.2f}°")
    #     self.get_logger().info(f"🔄 Rotating: {angle_diff:.2f}° | 📏 Moving: {distance:.3f}m")

    #     # **Step 1: Rotate**
    #     self.rotate_fixed(angle_diff, lambda: self.move_forward_fixed(distance))

    def move_to_next_waypoint(self):
        """Moves to the next waypoint using fixed rotation & movement in TurtleBot's local frame."""
        
        # ✅ Ensure we have position, yaw, and transformed waypoints
        if not self.ready or not self.square_waypoints:
            self.get_logger().warn("⚠️ Waiting for both position & orientation before moving.")
            return

        # ✅ Stop if all waypoints are completed
        if self.current_index >= len(self.square_waypoints):
            self.get_logger().info("🏁 Square Complete! Stopping.")
            self.stop_robot()
            return

        # ✅ Get next waypoint in TurtleBot's frame
        goal_x_local, goal_y_local, _ = self.square_waypoints[self.current_index]

        # ✅ Compute the target angle in TurtleBot’s local frame
        target_angle = math.degrees(math.atan2(goal_y_local, goal_x_local))

        # ✅ Compute the shortest rotation direction
        angle_diff = target_angle  # Already in local frame
        if angle_diff > 180:
            angle_diff -= 360
        elif angle_diff < -180:
            angle_diff += 360

        # ✅ Compute movement distance (TurtleBot frame)
        distance = math.sqrt(goal_x_local**2 + goal_y_local**2)

        self.get_logger().info(f"📍 Target (Local Turtlebot Frame): X={goal_x_local:.3f}, Y={goal_y_local:.3f}")
        self.get_logger().info(f"🧭 Current Yaw: {self.yaw:.2f}° | Target Angle (Local): {target_angle:.2f}°")
        self.get_logger().info(f"🔄 Rotating: {angle_diff:.2f}° | 📏 Moving: {distance:.3f}m")

        # **Step 1: Rotate first**
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



