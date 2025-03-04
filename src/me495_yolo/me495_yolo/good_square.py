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

        #subscriptions (subscribing to turtlebot position and turtlebot orientation and dancer waypoints in the camera frame)
        self.create_subscription(Float32MultiArray, '/turtlebot_position_april', self.position_callback, 10)
        self.create_subscription(Float32MultiArray, '/turtlebot_orientation_april', self.orientation_callback, 10)
        self.create_subscription(Float32MultiArray, '/turtlebot_front_april', self.front_callback, 10)
        self.create_subscription(Path, '/dancer_waypoints', self.dancer_waypoints_callback, 10)


        #publishers (publishing movement commands in the turtle frame )
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        #publisher for waypoints!!!
        self.waypoint_marker_pub = self.create_publisher(Marker, '/waypoint_marker', 10)

        #init some things
        self.robot_position = None #robot position from subscription
        self.yaw = None #robot orientation from subscription
        self.waypoints_camera = [] #empty list for waypoioints in camera frame
        self.square_waypoints = None #these are the waypoints i make 
        self.current_index = 0 #this is what waypoint i am at in the list of waypoints
        self.ready = False # boolean that will make sure i have waypoints in camera frame (i have both robot pos and orientation) 
        self.is_moving = False #to make sure we are getting position and orientation when turtltebot is stationary 
        self.waypoints_received = False


    def position_callback(self, msg):
        """Receives initial position of turtlebot in camera frame from subscription """
       
        self.robot_position = (msg.data[0], msg.data[1])
        #self.get_logger().info(f"📡 Turtlebot Position: X={self.robot_position[0]:.3f}, Y={self.robot_position[1]:.3f}")
            
        #breaks my robot position into an x and y value
        x, y = (msg.data[0], msg.data[1])

        if self.ready:
            self.move_to_next_waypoint()

    def dancer_waypoints_callback(self, msg):
        '''callback function that receives my dancer waypoints ONLY ONCE from waypoint april and stores them'''

        #dont get waypoints if i already have them 
        if self.waypoints_received:
            #self.get_logger().info("Already have waypoints!")
            return 
        
        #make sure i have waypoints
        if not msg.poses:
            #self.get_logger().warn("Received empty waypoint list")
            return 
        
        #if i dont already have waypoints: 
        if self.waypoints_received is False:
            #store waypoints and set flag 
            self.waypoints_camera= [(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z) for pose in msg.poses]
            self.get_logger().info(f"📥 Received {len(self.waypoints_camera)} waypoints from dancer!")
            self.waypoints_received = True 

            #start movement 
            self.current_index = 0
            self.ready = True
            self.publish_waypoint_marker()
            self.move_to_next_waypoint()

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
            self.get_logger().info(f"📍 Front AprilTag Position: X={self.turtlebot_front_position[0]:.3f}, Y={self.turtlebot_front_position[1]:.3f}")


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

    def move_to_next_waypoint(self):
        """ moves the turtlebot to the next waypoint using co linearity check """

        self.get_logger().info(f"📍 Current waypoint index: {self.current_index} / {len(self.waypoints_camera)-1}")

        #if i have gone through all the waypoints stop because path is complete
        if self.current_index > (len(self.waypoints_camera)-1):
            self.get_logger().info("🏁 No more waypoints. Stopping.")
            #self.stop_robot()
            self.cmd_vel_pub.publish(Twist())  # this means send all zeros 

            return
        
        #if there are still waypoints to go to lets go to the next goal waypoint
        #to go to next waypoint we first spin and then stop when colinear and then move forward 
        #we need the current waypoints position, the robot postion and the turtlebot front position
        Xw, Yw, _ = self.waypoints_camera[self.current_index]   
        Xb, Yb = self.robot_position
        Xf, Yf = self.turtlebot_front_position

        #now compute 2d cross product to check co linearity 
        cross_product = (Yf - Yb) * (Xw - Xf) - (Xf - Xb) * (Yw - Yf)

        #we need to compute distances from waypoint to check if turtlebot is facing towards or away
        dist_front_to_waypoint =  np.linalg.norm([Xw - Xf, Yw - Yf])
        dist_center_to_waypoint = np.linalg.norm([Xw - Xb, Yw - Yb])

        #logging 
        self.get_logger().info(f" Cross Product: {cross_product:.6f} (Closer to 0 means collinear)") 
        #self.get_logger().info(f"📍 Distance to Waypoint {self.current_index} | Front: {dist_front_to_waypoint:.3f}, Center: {dist_center_to_waypoint:.3f}")


        #if cross product is near 0 and the front is closer than the center, we move forward
        if (abs(cross_product)< 0.01) and (dist_front_to_waypoint < dist_center_to_waypoint): #little threshold to check co linearity
            #self.get_logger().info(f"Collinear with waypoint moving forward! ")
            self.cmd_vel_pub.publish(Twist())  # stop rotating when aligned --> this means send all zeros 
            self.move_forward_fixed() # then move froward 

        else:
            #self.get_logger().info(f" Rotating right.") #right now we are just rotating to the right always 
            self.rotate_fixed(-0.1)  # Rotate right
    
    def rotate_fixed(self, angular_speed):
            """Rotates the TurtleBot at a fixed speed until it aligns with the waypoint and co linearity condition is met"""

            #self.get_logger().info(f"🔄 Rotating at speed: {angular_speed:.2f} rad/s")
    
            # Create and publish twist message for rotation
            twist = Twist()
            twist.angular.z = angular_speed  # rotate left (positive) or right (negative)
            self.cmd_vel_pub.publish(twist)

    def move_forward_fixed(self, speed= -0.05): #trying negative speed to see if that means "forward"
        """Moves the TurtleBot forward at a fixed speed until it reaches the waypoint."""

        #self.get_logger().info(f"🚀 Moving forward at speed: {speed:.2f} m/s")

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

        if distance_to_waypoint < 0.09:
            self.get_logger().info(f"🏁 Reached waypoint {self.current_index}! Stopping.")
            self.cmd_vel_pub.publish(Twist())  # stop rotating when alined --> this means send all zeros 
            self.current_index += 1  # advance to next waypoint
            self.get_logger().info(f"INCREMENTING CURRENT INDEX")


        
            self.move_to_next_waypoint()  # start moving to the next waypoint


def main():
    rclpy.init()
    node = GoodSquareMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()






        


        







        




    



