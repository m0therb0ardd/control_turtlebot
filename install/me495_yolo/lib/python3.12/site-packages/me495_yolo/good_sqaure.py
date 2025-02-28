#imports 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import tf2_ros
import tf_transformations
import numpy as np 


class GoodSquareMover(Node):
    def __init__(self):
        #my node
        super().__init__('good_square_mover')

        #subscriptions (subscribing to turtlebot position and turtlebot orientation in the camera frame)
        self.create_subscription(Float32MultiArray, '/turtlebot_position_april', self.position_callback, 10)
        self.create_subscription(Float32MultiArray, '/turtlebot_orientation_april', self.orientation_callback, 10)
        self.create_subscription(Float32MultiArray, '/turtlebot_front_april', self.front_callback, 10)

        #publishers (publishing movement commands in the turtle frame )
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        #init some things
        self.robot_position = None #robot position from subscription
        self.yaw = None #robot orientation from subscription
        self.square_waypoints = None #these are the waypoints i make 
        self.current_index = 0 #this is what waypoint i am at in the list of waypoints
        self.ready = False # boolean that will make sure i have waypoints in turtle frame (i have both robot pos and orientation) 
        self.is_moving = False #to make sure we are getting position and orientation when turltebot is stationary 
        #for waypoint transformation 
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)



    def position_callback(self, msg):
        """Receives initial position of turtlebot in camera frame from subscription & sets waypoints in camera frame and then also converts waypoitns to turtlebto frame """
        if self.robot_position or self.square_waypoints is None :
            self.robot_position = (msg.data[0], msg.data[1])
            self.get_logger().info(f"📡 Initial Turtlebot Position: X={self.robot_position[0]:.3f}, Y={self.robot_position[1]:.3f}")
            
            #breaks my robot position into an x and y value
            x, y = (msg.data[0], msg.data[1])

            # start by just defining the first waypoint of a square movement 
            waypoints_camera = [
                (x + 0.132, y, 0.0),  # right
                # (x + 0.132, y + 0.132, 0.0),  # up
                # (x, y + 0.132, 0.0),  # left
                # (x, y, 0.0)  # down (back to start)
            ]

            self.get_logger().info(f"📍 Original Waypoints (World Frame): {waypoints_camera}")


            #if both necessary frames for turtle orientation and position exist we can transform the waypoints to be in turtlebot frame 
            self.square_waypoints = self.transform_waypoints_to_turtlebot(waypoints_camera)

        # check that waypoints have been transformed and we are ready to move 
        if self.yaw is not None and self.square_waypoints is not None:
            self.ready = True
            self.move_to_next_waypoint()

    def orientation_callback(self, msg):
        """Receives orientation directly from YOLO node."""
        if len(msg.data) >= 1:  # Ensure there's waypoint data
            self.yaw = msg.data[0]  # Store received yaw
            self.get_logger().info(f"🧭 Received TurtleBot Orientation from YOLO: {self.yaw:.2f}°")

            # Check if both position and waypoints in turtle frame are available
            if self.robot_position is not None and self.square_waypoints:
                self.ready = True
                self.move_to_next_waypoint()

    def front_callback(self, msg):
        """Receives TurtleBot's front position in the camera frame from apriltag_detection node."""
        if len(msg.data) >= 2:
            self.turtlebot_front_position = (msg.data[0], msg.data[1])  # Store (x, y) position
            self.get_logger().info(f"📍 Front AprilTag Position: X={self.turtlebot_front_position[0]:.3f}, Y={self.turtlebot_front_position[1]:.3f}")

    def transform_waypoints_to_turtlebot(self, waypoints):
            """Transforms waypoints from the camera frame to the TurtleBot’s local frame using TF lookups."""

            #start with an empty list of waypoints
            transformed_waypoints = []

            for x, y, z, in waypoints:
                try:
                    if not self.tf_buffer.can_transform("turtlebot_position_april", "camera_color_optical_frame", rclpy.time.Time()):
                        self.get_logger().warn("🚨 Transform NOT available yet. Skipping this transformation for now.")
                        return None  # Skip transformation & retry later   

                     # Proceed with transformation if all the frames i need are found
                    transform = self.tf_buffer.lookup_transform("turtlebot_position_april", "camera_color_optical_frame", rclpy.time.Time())
                    self.get_logger().info("✅ ✅ Performing transformation of wyapoitns to turtlebot frame.")

                    # Extract translation & rotation
                    trans = transform.transform.translation
                    rot = transform.transform.rotation

                    # Convert quaternion to a rotation matrix
                    quat = [rot.x, rot.y, rot.z, rot.w]
                    rot_matrix = tf_transformations.quaternion_matrix(quat)

                    # Convert waypoint to a homogeneous coordinate (4x1 vector)
                    point = np.array([x, y, z, 1]).reshape(4, 1)

                    # Apply transformation (Rotation + Translation)
                    transformed_point = np.dot(rot_matrix, point)
                    transformed_x = transformed_point[0, 0] + trans.x
                    transformed_y = transformed_point[1, 0] + trans.y
                    transformed_z = transformed_point[2, 0] + trans.z

                    self.get_logger().info(f"✅ Transformed Waypoints in Turtle Frame : X={transformed_x:.3f}, Y={transformed_y:.3f}, Z={transformed_z:.3f}")

                    #add transformed waypoints to my list opf waypoints
                    transformed_waypoints.append((transformed_x, transformed_y, transformed_z))

                except Exception as e:
                    self.get_logger().error(f"🚨 Failed to transform waypoint: {e} to turtlebot frame")

            return transformed_waypoints


    def orientation_callback(self, msg):
        """Receives orientation of turtlebot directly from YOLO node in a range of 0 to 360 degrees.
                                    CAMERA
                                     90 

                TOWARDS PUSHKAR                    RIGHT
                        0/360                       180

                                    TOWARDS JOE
                                    270
        
        
        """
        if len(msg.data) >= 1:  # Ensure there's data
            self.yaw = msg.data[0]  # Store received yaw
            self.get_logger().info(f"🧭 Received TurtleBot Orientation from YOLO: {self.yaw:.2f}°")

        # Check if both position and waypoints are available 
        if self.robot_position is not None and self.square_waypoints is not None:
            self.ready = True
            self.move_to_next_waypoint()

    def move_to_next_waypoint(self):
        """ moves the turtlebot to the next waypoint using co linearity check """

        #make sure i have waypoints in correct frame before moving 
        if not self.ready == True:
            self.get_logger().warn("⚠️ Waiting for both position & orientation and transformed waypoints before moving.")
            return
        
        # my self.current_index is set to 0 in init
        #make sure i still have waypoints to move to 
        if self.current_index >= len(self.square_waypoints):
            self.get_logger().info("🏁 No more waypoints. Stopping.")
            self.stop_robot()
            return
        
        #if there are still waypoints to go to lets go to the next goal waypoint
        Xw, Yw, _ = self.square_waypoints[self.current_index]

        #now we start to check for co linearity by first making sure i have the coords of turltebot center and adn turtlebot front
        if not hasattr(self, "turtlebot_front_position") or self.turtlebot_front_position is None:
                self.get_logger().warn("⚠️ No front position detected yet. Waiting...")
                return
        
        Xb, Yb = self.robot_position
        Xf, Yf = self.turtlebot_front_position

        #now compute cross product to check co linearity 
        cross_product = (Yf - Yb) * (Xw - Xf) - (Xf - Xb) * (Yw - Yf)

        #if cross product is near 0 we move forward
        if abs(cross_product)< 0.05: #little threshold to check co linearity
            self.get_logger().info(f"Collinear with waypoint! Moving forward.")
            self.cmd_vel_pub.publish(Twist())  # stop rotating when alined I THINK THIS MEANS IT SENDS ALL ZEROS 
            #self.move_forward_fixed() # then move froward COMMENTED OUT FOR NOW 

        elif cross_product > 0:
            self.get_logger().info(f"🔄 Waypoint is to the LEFT. Rotating left.")
            self.rotate_fixed(0.2)  # Rotate left

        else:
            self.get_logger().info(f"🔄 Waypoint is to the RIGHT. Rotating right.")
            self.rotate_fixed(-0.2)  # Rotate right
    
    def rotate_fixed(self, angular_speed):
            """Rotates the TurtleBot at a fixed speed until it aligns with the waypoint and co linearity condition is met"""

            self.get_logger().info(f"🔄 Rotating at speed: {angular_speed:.2f} rad/s")
    
            # Create and publish twist message for rotation
            twist = Twist()
            twist.angular.z = angular_speed  # rotate left (positive) or right (negative)
            self.cmd_vel_pub.publish(twist)

            # Keep rotating and rechecking collinearity
            self.create_timer(0.1, self.move_to_next_waypoint)

def main():
    rclpy.init()
    node = GoodSquareMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()






        


        







        




    



