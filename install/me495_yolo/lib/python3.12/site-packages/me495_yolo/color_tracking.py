import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math

class ColorTrackerNode(Node):
    def __init__(self):
        super().__init__('color_tracker_hsv')
        self.bridge = CvBridge()
        
        # ROS2 Subscriptions
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10)
        
        # ROS2 Publishers
        self.dancer_position_pub = self.create_publisher(Float32MultiArray, '/dancer_position_color', 10)
        self.turtlebot_position_pub = self.create_publisher(Float32MultiArray, '/turtlebot_position_color', 10)
        self.turtlebot_front_pub = self.create_publisher(Float32MultiArray, '/turtlebot_front_color', 10)
        self.turtlebot_orientation_pub = self.create_publisher(Float32MultiArray, '/turtlebot_orientation_color', 10)
        self.image_publisher = self.create_publisher(Image, '/new_image', 10)  # Added publisher for processed image
        
        # TF Broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Camera Intrinsics (to be updated when CameraInfo is received)
        self.fx, self.fy = None, None
        self.cx, self.cy = None, None
        
        # HSV Color Ranges
        self.pink_range = ((145, 50, 230), (165, 255, 255))  # Dancer
        self.blue_range = ((45, 40, 140), (90, 255, 255))  # TurtleBot
        #self.yellow_range = ((25, 100, 115), (30, 255, 255))  # TurtleBot Front FLY ZONE
        self.yellow_range = ((15, 122, 121), (63, 255, 255))

        self.get_logger().info("🚀 HSV Color Tracker Initialized")

    def camera_info_callback(self, msg):
        """Extract camera intrinsics from CameraInfo topic."""
        if self.fx is None:
            self.fx, self.fy = msg.k[0], msg.k[4]
            self.cx, self.cy = msg.k[2], msg.k[5]
            self.get_logger().info(f"📷 Camera Info Received: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}")

    def image_callback(self, msg):
        """Detect objects using HSV and publish their positions."""
        if self.fx is None:
            self.get_logger().warn("⚠️ Waiting for Camera Info before processing images!")
            return
        
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Detect Objects
        dancer_coords = self.detect_color(cv_image, hsv_image, self.pink_range, "Dancer", (255, 0, 255))
        turtlebot_coords = self.detect_color(cv_image, hsv_image, self.blue_range, "TurtleBot", (255, 255, 0))
        front_coords = self.detect_color(cv_image, hsv_image, self.yellow_range, "TurtleBot Front", (0, 255, 255))
        
        # Publish & Broadcast TFs
        if dancer_coords:
            self.publish_and_broadcast("dancer_position_color", self.dancer_position_pub, dancer_coords)
        
        if turtlebot_coords:
            self.publish_and_broadcast("turtlebot_position_color", self.turtlebot_position_pub, turtlebot_coords)
        
        if front_coords:
            self.publish_and_broadcast("turtlebot_front_color", self.turtlebot_front_pub, front_coords)
        
        # Compute Orientation if both TurtleBot & Front detected
        if turtlebot_coords and front_coords:
            self.compute_and_publish_orientation(turtlebot_coords, front_coords)
        
        # Publish the annotated image
        new_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        self.image_publisher.publish(new_msg)
    
    def detect_color(self, cv_image, hsv_image, color_range, label, draw_color):
        """Detects the largest contour of a given color and returns real-world coordinates."""
        mask = cv2.inRange(hsv_image, np.array(color_range[0]), np.array(color_range[1]))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None
        
        largest_contour = max(contours, key=cv2.contourArea)

         # Ignore small detections
        if cv2.contourArea(largest_contour) < 100:
            return None  
    
        moments = cv2.moments(largest_contour)
        
        if moments["m00"] == 0:
            return None
        
        cX = int(moments["m10"] / moments["m00"])
        cY = int(moments["m01"] / moments["m00"])
        
        # Draw detected object
        cv2.circle(cv_image, (cX, cY), 5, draw_color, -1)
        cv2.putText(cv_image, label, (cX + 10, cY - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, draw_color, 2)
        
        return self.pixel_to_world(cX, cY)
    
    def pixel_to_world(self, u, v):
        """Convert pixel coordinates (u, v) to real-world coordinates."""
        Z = 1.0  # Assume fixed depth for consistency
        X = (u - self.cx) * Z / self.fx
        Y = (v - self.cy) * Z / self.fy
        return (X, Y, Z)
    
    def publish_and_broadcast(self, frame_id, publisher, coords):
        """Publish coordinates as Float32MultiArray and broadcast TF."""
        msg = Float32MultiArray()
        msg.data = list(coords)
        publisher.publish(msg)
        
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'camera_color_optical_frame'
        t.child_frame_id = frame_id
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = coords
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)
    
    def compute_and_publish_orientation(self, base_coords, front_coords):
        """Compute yaw from TurtleBot base to front marker and publish it."""
        Xb, Yb, _ = base_coords
        Xy, Yy, _ = front_coords
        yaw = math.atan2(Yy - Yb, Xy - Xb)
        yaw_degrees = (math.degrees(yaw) + 360) % 360
        
        msg = Float32MultiArray()
        msg.data = [yaw_degrees]
        self.turtlebot_orientation_pub.publish(msg)

def main():
    rclpy.init()
    node = ColorTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
