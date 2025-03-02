#imports 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float32MultiArray
import tf2_ros
import tf_transformations
import numpy as np 
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path


''' little note to self that my entire waypoint system (movement, checking co-linearity adn visualization in rviz
happens in the camera frame although i do have code in here that transforms waypoints to turtlebot frame i dont actually use it)'''
class GoodSquareMover(Node):
    def __init__(self):
        #my node
        super().__init__('good_square_mover')

        #subscriptions (subscribing to turtlebot position and turtlebot orientation in the camera frame)
        self.create_subscription(Float32MultiArray, '/turtlebot_position_april', self.position_callback, 10)
        self.create_subscription(Float32MultiArray, '/turtlebot_orientation_april', self.orientation_callback, 10)
        self.create_subscription(Float32MultiArray, '/turtlebot_front_april', self.front_callback, 10)
        #adding a subscription to my waypints
        self.create_subscription(Path, '/dancer_waypoints', self.dancer_waypoints_callback, 10)
        self.get_logger().info("✅ Subscribed to /dancer_waypoints!")

        #publishers (publishing movement commands in the turtle frame )
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        #publisher for waypoints!!!
        self.waypoint_marker_pub = self.create_publisher(Marker, '/waypoint_marker', 10)

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
        self.waypoints_camera = []



    def position_callback(self, msg):
        """Receives initial position of turtlebot in camera frame from subscription & sets waypoints in camera frame and then also converts waypoitns to turtlebto frame """
        if (self.robot_position is None) and (self.square_waypoints is None) :
            #self.robot_position = (msg.data[0], msg.data[1])
            #self.get_logger().info(f"📡 Initial Turtlebot Position: X={self.robot_position[0]:.3f}, Y={self.robot_position[1]:.3f}")
            
            #breaks my robot position into an x and y value
            x, y = (msg.data[0], msg.data[1])

            # # start by just defining the first waypoint of a square movement 
            # self.waypoints_camera = [
            #     (x-0.14 , y-0.14, 1.0),  # right
            #     #(x - 0.14, y + 0.4, 1.0),  # up
            #     (x, y + 0.14, 1.0),  # left
            #     #(x, y, 1.0)  # down (back to start)
            # ]

            self.get_logger().info(f"📍 Original Waypoints (camera Frame): {self.waypoints_camera}")

            # Publish waypoints to RViz
            self.publish_waypoint_marker()

            #if both necessary frames for turtle orientation and position exist we can transform the waypoints to be in turtlebot frame 
            self.square_waypoints = self.transform_waypoints_to_turtlebot(self.waypoints_camera)

        # check that waypoints have been transformed and we are ready to move 
        if self.yaw is not None and self.square_waypoints is not None:
            self.robot_position = (msg.data[0], msg.data[1]) #only fill in robot pos if tranformation happens 
            #self.get_logger().info(f"📡 Initial Turtlebot Position: X={self.robot_position[0]:.3f}, Y={self.robot_position[1]:.3f}")
            self.current_index = 0
            self.ready = True
            self.move_to_next_waypoint()

    def dancer_waypoints_callback(self, msg):
        '''callback function that receives my dancer waypoints from waypoint april and stores them'''
        #am i entering callback?
        self.get_logger().info("📡 Callback triggered! Received message from /dancer_waypoints")
        
        #make sure i have waypoints
        if not msg.poses:
            self.get_logger().warn("Received empty waypoint list")
            return 
        
        self.waypoints_camera= [(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z) for pose in msg.poses]
        self.get_logger().info(f"📥 Received {len(self.waypoints_camera)} waypoints from dancer!")

        # Start moving once waypoints are received
        # self.current_index = 0
        # self.ready = True
        #self.move_to_next_waypoint()

    def publish_waypoint_marker(self):
        """Publishes waypoint markers to rviz to help with visualization """
        """CHAT GPT DID THIS ONE"""
        for i, waypoint in enumerate(self.waypoints_camera):
            marker = Marker()
            marker.header.frame_id = "camera_color_optical_frame" #sets reference frame
            marker.header.stamp = self.get_clock().now().to_msg() # time of frame created 
            marker.ns = "waypoints" #marker name space this groups togehter all the waypoints for when i have more than one 
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            # Set position of the waypoint in the camera frame
            marker.pose.position.x = waypoint[0]
            marker.pose.position.y = waypoint[1]
            marker.pose.position.z = waypoint[2]  

            # Appearance settings
            marker.scale.x = 0.1  # Sphere size
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.r = 1.0  # Red color
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0  # Fully visible

            self.waypoint_marker_pub.publish(marker)
            self.get_logger().info(f"📍 Published waypoint marker {i} at X={waypoint[0]:.3f}, Y={waypoint[1]:.3f}")



    def front_callback(self, msg):
        """Receives TurtleBot's front position in the camera frame from apriltag_detection node."""
        if len(msg.data) >= 2:
            self.turtlebot_front_position = (msg.data[0], msg.data[1])  # Store (x, y) position
            #self.get_logger().info(f"📍 Front AprilTag Position: X={self.turtlebot_front_position[0]:.3f}, Y={self.turtlebot_front_position[1]:.3f}")

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
            #self.get_logger().info(f"🧭 Received TurtleBot Orientation from YOLO: {self.yaw:.2f}°")

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
        self.get_logger().info(f"current index {self.current_index}")

        if self.current_index >= len(self.square_waypoints):
            self.get_logger().info("🏁 No more waypoints. Stopping.")
            #self.stop_robot()
            self.cmd_vel_pub.publish(Twist())  # this means send all zeros 

            return
        
        #if there are still waypoints to go to lets go to the next goal waypoint
        Xw, Yw, _ = self.waypoints_camera[self.current_index]

        #now we start to check for co linearity by first making sure i have the coords of turltebot center and adn turtlebot front
        if not hasattr(self, "turtlebot_front_position") or self.turtlebot_front_position is None:
                self.get_logger().warn("⚠️ No front position detected yet. Waiting...")
                return
        
        Xb, Yb = self.robot_position
        Xf, Yf = self.turtlebot_front_position

        #now compute 2d cross product to check co linearity 
        cross_product = (Yf - Yb) * (Xw - Xf) - (Xf - Xb) * (Yw - Yf)

        #we need to compute distances from waypoint to chekc if turtlebot is facing towards or away
        dist_front_to_waypoint =  np.linalg.norm([Xw - Xf, Yw - Yf])
        dist_center_to_waypoint = np.linalg.norm([Xw - Xb, Yw - Yb])

        #logging 
        self.get_logger().info(f" Cross Product: {cross_product:.6f} (Closer to 0 means collinear)") 
        self.get_logger().info(f"📍 Distance to Waypoint | Front: {dist_front_to_waypoint:.3f}, Center: {dist_center_to_waypoint:.3f}")


        #if cross product is near 0 we move forward
        if (abs(cross_product)< 0.01) and (dist_front_to_waypoint < dist_center_to_waypoint): #little threshold to check co linearity
            self.get_logger().info(f"Collinear with waypoint moving forward! ")
            self.cmd_vel_pub.publish(Twist())  # stop rotating when aligned --> this means send all zeros 
            self.move_forward_fixed() # then move froward 

        else:
            self.get_logger().info(f" Rotating right.") #right now we are just rotating to the right always 
            self.rotate_fixed(-0.1)  # Rotate right
    
    def rotate_fixed(self, angular_speed):
            """Rotates the TurtleBot at a fixed speed until it aligns with the waypoint and co linearity condition is met"""

            self.get_logger().info(f"🔄 Rotating at speed: {angular_speed:.2f} rad/s")
    
            # Create and publish twist message for rotation
            twist = Twist()
            twist.angular.z = angular_speed  # rotate left (positive) or right (negative)
            self.cmd_vel_pub.publish(twist)

    def move_forward_fixed(self, speed= -0.1): #trying negative speed to see if that means "forward"
        """Moves the TurtleBot forward at a fixed speed until it reaches the waypoint."""

        self.get_logger().info(f"🚀 Moving forward at speed: {speed:.2f} m/s")

        twist = Twist()
        twist.linear.x = speed
        self.cmd_vel_pub.publish(twist)

        self.create_timer(0.1, self.check_reached_waypoint)

    def check_reached_waypoint(self):
        Xw, Yw, _ = self.waypoints_camera[self.current_index]  # get coords of current waypoint (this is in camera frame)
        Xb, Yb = self.robot_position  # get TurtleBot center position (this is in camera frame)

        #compute distance btwn turtlebot center and waypoint 
        distance_to_waypoint = np.linalg.norm([Xw -Xb, Yw - Yb])

        self.get_logger().info(f"📏 Distance to Waypoint: {distance_to_waypoint:.3f}m")

        #if im at the waypoint (or close enough to it) stop and move to next waypoint which means incrementing my current_index

        if distance_to_waypoint < 0.05:
            self.get_logger().info(f"🏁 Reached waypoint {self.current_index}! Stopping.")
            self.cmd_vel_pub.publish(Twist())  # stop rotating when alined --> this means send all zeros 
            self.current_index += 1  # advance to next waypoint
            self.move_to_next_waypoint()  # start moving to the next waypoint


def main():
    rclpy.init()
    node = GoodSquareMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()






        


        







        




    



