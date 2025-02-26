import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import MarkerArray
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
from me495_yolo.waypoint_follower import TurtleBotWaypointFollower
import math
import tf_transformations

class YoloNode(Node):
    def __init__(self):
        super().__init__("color_tracker")
        self.bridge = CvBridge()

    ## make straigtht hahead 180 and test gets rird of flipping problem 
    ## if clsoe to 360 0r 0 move by 10 degrees 
    ## 
    ## proprortional term for turning speed and forward to change 
    ## 
    ## 
    ## play around with exposure time and make sure autoexpossure turned off 
    ## how does autoexposure work 

        #APRIL TAG SUBSCRIPTIONS 
        self.get_logger().info("🚀 YoloNode started! Subscribing to /dancer_position_april")

        self.create_subscription(Float32MultiArray, "/turtlebot_position_april", self.turtlebot_position_callback, 10)
        self.create_subscription(Float32MultiArray, "/turtlebot_orientation_april", self.turtlebot_orientation_callback, 10)
        self.create_subscription(Float32MultiArray, "/dancer_position_april", self.dancer_position_callback, 10)
       
        # Camera subscription
        self.create_subscription(Image, "/camera/camera/color/image_raw", self.image_callback, 10)
        self.image_publisher = self.create_publisher(Image, "/new_image", 10)  # Publish processed image
        self.get_logger().info("📷 Subscribed to /camera/camera/color/image_raw")


        # Timer variables
        self.start_time = None  # Store start time of movement
        self.time_limit = 8 # Stop recording waypoints after 5 seconds
        #self.tf_timer = self.create_timer(0.1, self.broadcast_all_tf)  # Broadcast every 100ms


        self.last_valid_dancer_pose = None  # Store last valid dancer position


        self.path_publisher = self.create_publisher(Float32MultiArray, 'path_points', 10)


        self.depth_image = None  # Store the latest depth frame
        self.fx, self.fy, self.cx, self.cy = None, None, None, None  # Camera intrinsics


        # Path storage
        self.path = []
        self.waypoint_markers = []
        self.dancer_path = []
        self.turtlebot_path = []

    def camera_info_callback(self, msg):
        """Extract camera intrinsics from CameraInfo topic."""
        if self.fx is None:
            self.fx, self.fy = msg.k[0], msg.k[4]  # Focal lengths
            self.cx, self.cy = msg.k[2], msg.k[5]  # Optical center
            self.get_logger().info(f" 📷 Camera Info Received: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}")

    def depth_callback(self, msg):
        """Store the latest depth frame and print a sample value."""
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.get_logger().info(f"📷 Depth Image Received: First Depth Value = {self.depth_image[240, 320]}")  # Sample at center pixel

    def image_callback(self, msg):
        """Receive camera images, detect AprilTags, and publish visualization."""
        self.get_logger().info("📸 Received an image from the camera")

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")  # Convert ROS2 Image to OpenCV
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)  # Convert to grayscale

        # **AprilTag Detection (You need an AprilTag library installed)**
        detector = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)  # Adjust dictionary as needed
        parameters = cv2.aruco.DetectorParameters()
        #corners, ids, _ = cv2.aruco.detectMarkers(gray, detector, parameters=parameters)
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
        corners, ids, _ = detector.detectMarkers(gray)

        # corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)


        if ids is not None:
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)  # Draw detected markers
            self.get_logger().info(f"✅ AprilTags detected: {ids.flatten()}")

        # Convert the processed image back to a ROS message and publish
        image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        self.image_publisher.publish(image_msg)
        self.get_logger().info("📤 Published processed image with AprilTags to /new_image")


    def turtlebot_orientation_callback(self, msg):
        """Callback for receiving TurtleBot orientation."""
        if len(msg.data) >= 1:
            yaw_degrees = msg.data[0]
            self.get_logger().info(f"🧭 Received TurtleBot Orientation: {yaw_degrees:.2f}°")

    
    def track_dancer_path(self, Xp, Yp, Zp=0.0):  # Ensure Z=0 if not provided
        """Adds dancer waypoints if movement is significant, publishes path."""
        ####### RIGHT NOW I AM ONLY GETTIGN DANCERRRR PATH IF IT MOVEESSSSSSSSSSSS
        
        if len(self.dancer_path) == 0 or (abs(Xp - self.dancer_path[-1][0]) > 0.00 or abs(Yp - self.dancer_path[-1][1]) > 0.05):
            self.dancer_path.append((Xp, Yp, Zp))  # Store full (x, y, z)

            self.get_logger().info(f"📍 Added Dancer Waypoint: ({Xp:.3f}, {Yp:.3f}, {Zp:.3f})")

        elapsed_time = time.time() - self.start_time if self.start_time else 0

        if elapsed_time > self.time_limit and self.dancer_path:
            self.get_logger().info(f"📤 Publishing {len(self.dancer_path)} waypoints!")
            
            # **Ensure Z=0 for all points**
            path_msg = Float32MultiArray()
            path_msg.data = [coord for point in self.dancer_path for coord in (point[0], point[1], 0.0)]
            self.path_publisher.publish(path_msg)

            self.get_logger().info(f"✅ Published {len(self.dancer_path)} points to /path_points")

            # Reset for next sequence
            self.dancer_path = []  
            self.start_time = None

    def dancer_position_callback(self, msg):
        """Receives dancer position and ensures it includes X, Y, Z."""
        if len(msg.data) >= 2:  # Minimum required is X and Y
            Xp = msg.data[0]
            Yp = msg.data[1]
            Zp = msg.data[2] if len(msg.data) > 2 else 0.0  # Default Z to 0.0 if missing

            self.get_logger().info(f"💃 Dancer Position Received: X={Xp:.3f}, Y={Yp:.3f}, Z={Zp:.3f}")

            # Store for path generation
            self.track_dancer_path(Xp, Yp, Zp)

            # **Ensure we always publish (X, Y, Z)**
            path_msg = Float32MultiArray()
            path_msg.data = [Xp, Yp, Zp]  # ✅ Now always three values
            self.path_publisher.publish(path_msg)

            self.get_logger().info(f"📤 Published path point: ({Xp:.3f}, {Yp:.3f}, {Zp:.3f}) to /path_points")


    def turtlebot_position_callback(self, msg):
        """Updates the TurtleBot's last known position and calls track_turtlebot_position()."""
        if len(msg.data) >= 3:
            Xb, Yb, Zb = msg.data[0], msg.data[1], msg.data[2]
            self.track_turtlebot_position(Xb, Yb, Zb)

    def compute_dancer_in_turtlebot_frame(self):
        """Compute dancer’s position in the TurtleBot’s reference frame using TF."""
        try:

            transform = self.tf_buffer.lookup_transform('turtlebot_position_april', 'dancer_position_april', rclpy.time.Time())
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z

            self.last_valid_dancer_pose = (x, y, z)
            self.get_logger().info(f"✅ Dancer in TurtleBot Frame: X={x:.3f}, Y={y:.3f}, Z={z:.3f}")

            return x, y, z
        except Exception as e:
            self.get_logger().warn(f"⚠️ Failed to compute dancer in TurtleBot frame: {e}")
            return self.last_valid_dancer_pose if self.last_valid_dancer_pose else None

def main():
    rclpy.init()
    node = YoloNode()
    rclpy.spin(node)
    rclpy.shutdown()


