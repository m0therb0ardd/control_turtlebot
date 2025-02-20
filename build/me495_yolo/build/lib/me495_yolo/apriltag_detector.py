# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2
# import apriltag
# from visualization_msgs.msg import Marker
# from geometry_msgs.msg import Point, PoseStamped
# from std_msgs.msg import ColorRGBA
# import tf_transformations

# class AprilTagDetector(Node):
#     def __init__(self):
#         super().__init__('apriltag_detector')
        
#         # Initialize CV Bridge
#         self.bridge = CvBridge()
        
#         # Initialize AprilTag detector
#         self.detector = apriltag.Detector()

#         # Subscribe to the image topic
#         self.subscription = self.create_subscription(
#             Image,
#             '/camera/camera/color/image_raw',
#             self.image_callback,
#             10)

#         # Publisher for visualization markers
#         self.marker_pub = self.create_publisher(Marker, '/apriltag_markers', 10)

#         self.get_logger().info("AprilTag Detector Node has started.")

#     def image_callback(self, msg):
#         try:
#             # Convert ROS Image message to OpenCV image
#             cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
#             # Convert to grayscale for AprilTag detection
#             gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
#             # Detect AprilTags
#             detections = self.detector.detect(gray_image)

#             if detections:
#                 self.get_logger().info(f"Detected {len(detections)} AprilTags.")
#                 for detection in detections:
#                     tag_id = detection.tag_id
#                     center_x, center_y = detection.center
#                     self.publish_marker(tag_id, center_x, center_y)
                    
#                     # Draw detection on image
#                     for idx in range(4):
#                         start_point = tuple(map(int, detection.corners[idx]))
#                         end_point = tuple(map(int, detection.corners[(idx + 1) % 4]))
#                         cv2.line(cv_image, start_point, end_point, (0, 255, 0), 2)
                    
#                     cv2.putText(cv_image, str(detection.tag_id), 
#                                 tuple(map(int, detection.center)), 
#                                 cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            
#             # Display image for debugging
#             cv2.imshow("AprilTag Detection", cv_image)
#             cv2.waitKey(1)

#         except Exception as e:
#             self.get_logger().error(f"Error processing image: {e}")

#     def publish_marker(self, tag_id, x, y):
#         """Publish a visualization marker for AprilTag detection."""
#         marker = Marker()
#         marker.header.frame_id = "camera_link"  # Use correct frame for your camera
#         marker.header.stamp = self.get_clock().now().to_msg()
#         marker.ns = "apriltags"
#         marker.id = tag_id
#         marker.type = Marker.SPHERE
#         marker.action = Marker.ADD

#         # Convert pixel coordinates to approximate 3D position (Assuming fixed depth)
#         marker.pose.position.x = x / 100.0  # Scale appropriately
#         marker.pose.position.y = y / 100.0  # Scale appropriately
#         marker.pose.position.z = 0.5  # Assume tags are at 0.5m depth

#         marker.pose.orientation.w = 1.0  # No rotation for now

#         # Marker appearance
#         marker.scale.x = 0.1  # Adjust size
#         marker.scale.y = 0.1
#         marker.scale.z = 0.1
#         marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  # Green color

#         self.marker_pub.publish(marker)
#         self.get_logger().info(f"Published marker for tag {tag_id}")

# def main():
#     rclpy.init()
#     apriltag_detector = AprilTagDetector()
#     rclpy.spin(apriltag_detector)
#     apriltag_detector.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


# import rclpy
# from rclpy.node import Node
# from apriltag_msgs.msg import AprilTagDetectionArray
# from std_msgs.msg import Float32MultiArray
# import math

# class AprilTagDetector(Node):
#     def __init__(self):
#         super().__init__('apriltag_detector')

#         # Subscribe to AprilTag detections
#         self.create_subscription(
#             AprilTagDetectionArray, 
#             '/apriltag_detections', 
#             self.apriltag_callback, 
#             10
#         )

#         # Publishers for TurtleBot and Dancer positions
#         self.turtlebot_position_pub = self.create_publisher(Float32MultiArray, '/turtlebot_position', 10)
#         self.turtlebot_orientation_pub = self.create_publisher(Float32MultiArray, '/turtlebot_orientation', 10)
#         self.dancer_position_pub = self.create_publisher(Float32MultiArray, '/dancer_position', 10)

#     def apriltag_callback(self, msg):
#         """Processes AprilTag detections and publishes positions."""
#         if not msg.detections:
#             self.get_logger().info("❌ No AprilTags detected.")
#             return

#         turtlebot_position = None
#         turtlebot_orientation = None
#         dancer_position = None

#         for detection in msg.detections:
#             tag_id = detection.id[0]  # Extract Tag ID
#             pose = detection.pose.pose.pose
#             x, y, z = pose.position.x, pose.position.y, pose.position.z
            
#             if tag_id == 11.4:  # TurtleBot Center
#                 turtlebot_position = [x, y]
#                 self.get_logger().info(f"🐢 TurtleBot Center: X={x:.2f}, Y={y:.2f}")
            
#             elif tag_id == 11.3:  # TurtleBot Front (for orientation)
#                 turtlebot_orientation = [x, y]
#                 self.get_logger().info(f"🐢 TurtleBot Front: X={x:.2f}, Y={y:.2f}")

#             elif tag_id == 11.0:  # Dancer
#                 dancer_position = [x, y]
#                 self.get_logger().info(f"💃 Dancer Position: X={x:.2f}, Y={y:.2f}")

#         # Publish TurtleBot's position and orientation
#         if turtlebot_position:
#             msg = Float32MultiArray()
#             msg.data = turtlebot_position
#             self.turtlebot_position_pub.publish(msg)

#         if turtlebot_orientation:
#             msg = Float32MultiArray()
#             msg.data = turtlebot_orientation
#             self.turtlebot_orientation_pub.publish(msg)

#         if dancer_position:
#             msg = Float32MultiArray()
#             msg.data = dancer_position
#             self.dancer_position_pub.publish(msg)

# def main():
#     rclpy.init()
#     node = AprilTagDetector()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# import rclpy
# from rclpy.node import Node
# from apriltag_msgs.msg import AprilTagDetectionArray
# from std_msgs.msg import Float32MultiArray
# from sensor_msgs.msg import Image, CameraInfo
# from message_filters import ApproximateTimeSynchronizer, Subscriber
# from cv_bridge import CvBridge
# import apriltag
# import math
# import cv2

# class AprilTagDetector(Node):
#     def __init__(self):
#         super().__init__('apriltag_detector')

#         # Initialize CV Bridge for image conversion
#         self.bridge = CvBridge()
        
#         # Initialize AprilTag detector
#         self.detector = apriltag.Detector()

#         # Synchronize image and camera info
#         self.image_sub = Subscriber(self, Image, "/camera/camera/color/image_raw")
#         self.info_sub = Subscriber(self, CameraInfo, "/camera/camera/color/camera_info")

#         self.sync = ApproximateTimeSynchronizer([self.image_sub, self.info_sub], queue_size=10, slop=0.1)
#         self.sync.registerCallback(self.image_callback)

#         # Subscribe to AprilTag detections
#         self.create_subscription(
#             AprilTagDetectionArray, 
#             '/apriltag_detections', 
#             self.apriltag_callback, 
#             10
#         )

#         # Publishers for TurtleBot and Dancer positions
#         self.turtlebot_position_pub = self.create_publisher(Float32MultiArray, '/turtlebot_position', 10)
#         self.turtlebot_orientation_pub = self.create_publisher(Float32MultiArray, '/turtlebot_orientation', 10)
#         self.dancer_position_pub = self.create_publisher(Float32MultiArray, '/dancer_position', 10)

#         self.get_logger().info("✅ AprilTag Detector Node Initialized.")

#     def image_callback(self, image_msg, camera_info_msg):
#         """Processes synchronized image and camera info."""
#         try:
#             # Convert ROS image to OpenCV image
#             cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
            
#             # Convert to grayscale for detection
#             gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

#             # Detect AprilTags
#             detections = self.detector.detect(gray_image)

#             if detections:
#                 self.get_logger().info(f"📸 Detected {len(detections)} AprilTags in Image.")
#                 for detection in detections:
#                     self.get_logger().info(f"🟢 Tag ID: {detection.tag_id}, Center: {detection.center}")

#         except Exception as e:
#             self.get_logger().error(f"⚠️ Error processing image: {e}")

#     def apriltag_callback(self, msg):
#         """Processes AprilTag detections and publishes positions."""
#         if not msg.detections:
#             self.get_logger().info("❌ No AprilTags detected.")
#             return

#         turtlebot_position = None
#         turtlebot_orientation = None
#         dancer_position = None

#         for detection in msg.detections:
#             tag_id = detection.id[0]  # Extract Tag ID
#             pose = detection.pose.pose.pose
#             x, y, z = pose.position.x, pose.position.y, pose.position.z

#             if tag_id == 11.4:  # TurtleBot Center
#                 turtlebot_position = [x, y]
#                 self.get_logger().info(f"🐢 TurtleBot Center: X={x:.2f}, Y={y:.2f}")

#             elif tag_id == 11.3:  # TurtleBot Front (for orientation)
#                 turtlebot_orientation = [x, y]
#                 self.get_logger().info(f"🐢 TurtleBot Front: X={x:.2f}, Y={y:.2f}")

#             elif tag_id == 11.0:  # Dancer
#                 dancer_position = [x, y]
#                 self.get_logger().info(f"💃 Dancer Position: X={x:.2f}, Y={y:.2f}")

#         # Publish positions if detected
#         if turtlebot_position:
#             msg = Float32MultiArray()
#             msg.data = turtlebot_position
#             self.turtlebot_position_pub.publish(msg)

#         if turtlebot_orientation:
#             msg = Float32MultiArray()
#             msg.data = turtlebot_orientation
#             self.turtlebot_orientation_pub.publish(msg)

#         if dancer_position:
#             msg = Float32MultiArray()
#             msg.data = dancer_position
#             self.dancer_position_pub.publish(msg)

# def main():
#     rclpy.init()
#     node = AprilTagDetector()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from apriltag_msgs.msg import AprilTagDetectionArray
import math

class AprilTagDetector(Node):
    def __init__(self):
        super().__init__('apriltag_detector')

        # Subscribe to AprilTag detections
        self.create_subscription(
            AprilTagDetectionArray, 
            '/detections',  # Updated to correct topic
            self.apriltag_callback, 
            10
        )

        # Publishers for TurtleBot and Dancer positions
        self.turtlebot_position_pub = self.create_publisher(Float32MultiArray, '/turtlebot_position_april', 10)
        self.turtlebot_orientation_pub = self.create_publisher(Float32MultiArray, '/turtlebot_orientation_april', 10)
        self.dancer_position_pub = self.create_publisher(Float32MultiArray, '/dancer_position_april', 10)

    # def apriltag_callback(self, msg):
    #     """Processes AprilTag detections and publishes positions."""
    #     if not msg.detections:
    #         self.get_logger().info("❌ No AprilTags detected.")
    #         return

    #     turtlebot_position = None
    #     turtlebot_orientation = None
    #     dancer_position = None

    #     for detection in msg.detections:
    #         tag_id = detection.id # Extract Tag ID
    #         center_x = detection.centre.x  # Pixel X coordinate of tag center
    #         center_y = detection.centre.y  # Pixel Y coordinate of tag center

    #         # Convert to meters (if needed, you may need camera calibration)
    #         x, y = center_x / 1000, center_y / 1000  # Example conversion from pixels to meters

    #         # Assign detections to corresponding tracked objects
    #         if tag_id == 11.4:  # TurtleBot Center
    #             turtlebot_position = [x, y]
    #             self.get_logger().info(f"🐢 TurtleBot Center: X={x:.2f}, Y={y:.2f}")
            
    #         elif tag_id == 11.3:  # TurtleBot Front (for orientation)
    #             turtlebot_orientation = [x, y]
    #             self.get_logger().info(f"🐢 TurtleBot Front: X={x:.2f}, Y={y:.2f}")

    #         elif tag_id == 11.0:  # Dancer
    #             dancer_position = [x, y]
    #             self.get_logger().info(f"💃 Dancer Position: X={x:.2f}, Y={y:.2f}")

    #     # Publish TurtleBot's position and orientation
    #     if turtlebot_position:
    #         msg = Float32MultiArray()
    #         msg.data = turtlebot_position
    #         self.turtlebot_position_pub.publish(msg)

    #     if turtlebot_orientation:
    #         msg = Float32MultiArray()
    #         msg.data = turtlebot_orientation
    #         self.turtlebot_orientation_pub.publish(msg)

    #     if dancer_position:
    #         msg = Float32MultiArray()
    #         msg.data = dancer_position
    #         self.dancer_position_pub.publish(msg)

    def apriltag_callback(self, msg):
        """Processes AprilTag detections and publishes positions."""
        self.get_logger().info(f"📡 Received {len(msg.detections)} detections")  # Print count

        if not msg.detections:
            self.get_logger().warn("❌ No AprilTags detected!")
            return

        for detection in msg.detections:
            tag_id = detection.id  # Get the tag ID
            x, y = detection.centre.x / 1000, detection.centre.y / 1000  # Convert mm → meters

            self.get_logger().info(f"🔍 Detected AprilTag {tag_id}: X={x:.2f}, Y={y:.2f}")

            # Check if tag IDs match expected values
            if tag_id == 4:  # TurtleBot Center
                pos_msg = Float32MultiArray(data=[x, y])
                self.turtlebot_position_pub.publish(pos_msg)
                self.get_logger().info(f"📡 Published TurtleBot Position: {x:.2f}, {y:.2f}")

            elif tag_id == 3:  # TurtleBot Front
                orient_msg = Float32MultiArray(data=[x, y])
                self.turtlebot_orientation_pub.publish(orient_msg)
                self.get_logger().info(f"🧭 Published TurtleBot Orientation: {x:.2f}, {y:.2f}")

            elif tag_id == 0:  # Dancer
                dancer_msg = Float32MultiArray(data=[x, y])
                self.dancer_position_pub.publish(dancer_msg)
                self.get_logger().info(f"💃 Published Dancer Position: {x:.2f}, {y:.2f}")


def main():
    rclpy.init()
    node = AprilTagDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
