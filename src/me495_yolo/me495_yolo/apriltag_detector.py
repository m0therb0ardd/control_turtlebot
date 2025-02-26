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
# ##################
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32MultiArray
# from apriltag_msgs.msg import AprilTagDetectionArray
# import math

# class AprilTagDetector(Node):
#     def __init__(self):
#         super().__init__('apriltag_detector')

#         # Subscribe to AprilTag detections
#         self.create_subscription(
#             AprilTagDetectionArray, 
#             '/detections',  # Updated to correct topic
#             self.apriltag_callback, 
#             10
#         )

#         # Publishers for TurtleBot and Dancer positions
#         self.turtlebot_position_pub = self.create_publisher(Float32MultiArray, '/turtlebot_position_april', 10)
#         self.turtlebot_orientation_pub = self.create_publisher(Float32MultiArray, '/turtlebot_orientation_april', 10)
#         self.dancer_position_pub = self.create_publisher(Float32MultiArray, '/dancer_position_april', 10)



#     def apriltag_callback(self, msg):
#         """Processes AprilTag detections and publishes positions."""
#         self.get_logger().info(f"📡 Received {len(msg.detections)} detections")  # Print count

#         if not msg.detections:
#             self.get_logger().warn("❌ No AprilTags detected!")
#             return

#         for detection in msg.detections:
#             tag_id = detection.id  # Get the tag ID
#             x, y = detection.centre.x / 1000, detection.centre.y / 1000  # Convert mm → meters

#             self.get_logger().info(f"🔍 Detected AprilTag {tag_id}: X={x:.2f}, Y={y:.2f}")

#             # Check if tag IDs match expected values
#             if tag_id == 4:  # TurtleBot Center
#                 pos_msg = Float32MultiArray(data=[x, y])
#                 self.turtlebot_position_pub.publish(pos_msg)
#                 self.get_logger().info(f"📡 Published TurtleBot Position: {x:.2f}, {y:.2f}")

#             elif tag_id == 3:  # TurtleBot Front
#                 orient_msg = Float32MultiArray(data=[x, y])
#                 self.turtlebot_orientation_pub.publish(orient_msg)
#                 self.get_logger().info(f"🧭 Published TurtleBot Orientation: {x:.2f}, {y:.2f}")

#             elif tag_id == 0:  # Dancer
#                 dancer_msg = Float32MultiArray(data=[x, y])
#                 self.dancer_position_pub.publish(dancer_msg)
#                 self.get_logger().info(f"💃 Published Dancer Position: {x:.2f}, {y:.2f}")


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
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math
import message_filters 
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2


class AprilTagDetector(Node):
    def __init__(self):
        super().__init__('apriltag_detector')

        self.bridge = CvBridge() 

        # Publishers for TurtleBot and Dancer positions
        self.turtlebot_position_pub = self.create_publisher(Float32MultiArray, '/turtlebot_position_april', 10)
        self.turtlebot_orientation_pub = self.create_publisher(Float32MultiArray, '/turtlebot_orientation_april', 10)
        self.dancer_position_pub = self.create_publisher(Float32MultiArray, '/dancer_position_april', 10)
        self.image_publisher = self.create_publisher(Image, "/new_image", 10)
        self.apriltag_sub = self.create_subscription(AprilTagDetectionArray, '/detections',  self.apriltag_callback, 10 )
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10)




        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ✅ Use message_filters to synchronize image and camera info topics
        image_sub = message_filters.Subscriber(self, Image, "/camera/camera/color/image_raw")
        info_sub = message_filters.Subscriber(self, CameraInfo, "/camera/camera/color/camera_info")

        # Approximate Time Synchronization (allows slight mismatch)
        self.ts = message_filters.ApproximateTimeSynchronizer([image_sub, info_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.synced_callback)

        self.fx, self.fy = None, None  # Focal lengths
        self.cx, self.cy = None, None  # Optical center


        self.get_logger().info("🚀 AprilTag detector initialized with synchronized image & camera_info!")


    def synced_callback(self, image_msg, camera_info_msg):
            """This function runs only when image & camera_info are received together."""
            self.get_logger().info("📸 Image & Camera Info received together ✅")

            # Convert ROS2 image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")

            # Draw debug text to confirm the image is being processed
            cv2.putText(cv_image, "AprilTag Processing Active", (50, 50), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # Convert back to ROS2 Image message and publish
            new_image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            self.image_publisher.publish(new_image_msg)

            self.get_logger().info("📤 Published camera image with AprilTag processing")

    def camera_info_callback(self, msg):
        """Extract camera intrinsics from CameraInfo topic."""
        if self.fx is None:  # Set values only once
            self.fx, self.fy = msg.k[0], msg.k[4]  # Focal lengths
            self.cx, self.cy = msg.k[2], msg.k[5]  # Optical center
            self.get_logger().info(f"📷 Camera Info Received in AprilTag Node: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}")



    def apriltag_callback(self, msg):
        """Processes AprilTag detections and publishes positions + TF frames."""
        self.get_logger().info(f"📡 Received {len(msg.detections)} detections")  # Print count

        if not msg.detections:
            self.get_logger().warn("❌ No AprilTags detected!")
            return

        turtlebot_base = None  # Stores (Xb, Yb, Zb) for TurtleBot center
        turtlebot_front = None  # Stores (Xy, Yy, Zy) for TurtleBot front
        dancer_position = None  # Stores (Xd, Yd, Zd) for Dancer

        for detection in msg.detections:
            tag_id = detection.id  # Get the tag ID
            Z = 1.0  # Assume fixed depth for consistency
            X = (detection.centre.x - self.cx) * Z / self.fx
            Y = (detection.centre.y - self.cy) * Z / self.fy

            self.get_logger().info(f"🔍 Detected AprilTag {tag_id}: X={X:.2f}, Y={Y:.2f}")

            if tag_id == 4:  # TurtleBot Center
                turtlebot_base = (X, Y, Z)
                pos_msg = Float32MultiArray(data=[X, Y, Z])  
                self.turtlebot_position_pub.publish(pos_msg)
                self.broadcast_tf("turtlebot_position_april", X, Y, Z)
                self.get_logger().info(f"📡 Published TurtleBot Position: {X:.2f}, {Y:.2f}")

            elif tag_id == 3:  # TurtleBot Front
                turtlebot_front = (X, Y, Z)
                self.broadcast_tf("turtlebot_front_april", X, Y, Z)
                self.get_logger().info(f"🧭 Published TurtleBot Front Position: {X:.2f}, {Y:.2f}")

            elif tag_id == 0:  # Dancer Position
                dancer_position = (X, Y, Z)
                dancer_msg = Float32MultiArray(data=[X, Y, Z])
                self.dancer_position_pub.publish(dancer_msg)
                self.broadcast_tf("dancer_position_april", X, Y, Z)
                self.get_logger().info(f"💃 Published Dancer Position: {X:.2f}, {Y:.2f}")

        # ✅ Compute Yaw **ONLY IF** both TurtleBot center & front are detected
        if turtlebot_base and turtlebot_front:
            Xb, Yb, _ = turtlebot_base
            Xy, Yy, _ = turtlebot_front

            # Compute yaw using atan2, same as color tracking
            dx = Xy - Xb  
            dy = Yy - Yb  
            yaw = math.atan2(dy, dx)  
            yaw_degrees = math.degrees(yaw)
            yaw_degrees = (yaw_degrees + 360) % 360  # Normalize to 0-360 degrees

            # Publish yaw
            orient_msg = Float32MultiArray(data=[yaw_degrees])  
            self.turtlebot_orientation_pub.publish(orient_msg)
            self.get_logger().info(f"🧭 Published TurtleBot Yaw (AprilTags): {yaw_degrees:.2f}°")


    def broadcast_tf(self, frame_id, x, y, z):
        """Broadcasts TF transform from camera to detected AprilTag."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "camera_color_optical_frame"  # Assuming camera frame
        t.child_frame_id = frame_id  # Assign unique frame name

        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)

        # Set rotation to identity quaternion (no rotation assumed)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = AprilTagDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
