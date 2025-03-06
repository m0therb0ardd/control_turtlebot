import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class HSVTunerNode(Node):
    def __init__(self):
        super().__init__("hsv_tuner")
        self.bridge = CvBridge()

        # Subscribe to RealSense camera feed
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        self.get_logger().info("📷 Subscribed to /camera/camera/color/image_raw")

        # Publisher for filtered HSV image
        self.hsv_pub = self.create_publisher(Image, '/hsv_filtered_image', 10)
        self.get_logger().info("📤 Publishing filtered image to /hsv_filtered_image")

        # Create trackbars for HSV tuning
        self.create_hsv_trackbars()

    def create_hsv_trackbars(self):
        """Creates OpenCV trackbars to adjust HSV values dynamically."""
        cv2.namedWindow("HSV Tuner", cv2.WINDOW_NORMAL)

        cv2.createTrackbar("H Min", "HSV Tuner", 0, 179, lambda x: None)
        cv2.createTrackbar("S Min", "HSV Tuner", 0, 255, lambda x: None)
        cv2.createTrackbar("V Min", "HSV Tuner", 0, 255, lambda x: None)

        cv2.createTrackbar("H Max", "HSV Tuner", 179, 179, lambda x: None)
        cv2.createTrackbar("S Max", "HSV Tuner", 255, 255, lambda x: None)
        cv2.createTrackbar("V Max", "HSV Tuner", 255, 255, lambda x: None)

    def image_callback(self, msg):
        """Processes incoming camera frames and applies HSV filtering."""
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Get HSV range from sliders
        h_min = cv2.getTrackbarPos("H Min", "HSV Tuner")
        s_min = cv2.getTrackbarPos("S Min", "HSV Tuner")
        v_min = cv2.getTrackbarPos("V Min", "HSV Tuner")
        h_max = cv2.getTrackbarPos("H Max", "HSV Tuner")
        s_max = cv2.getTrackbarPos("S Max", "HSV Tuner")
        v_max = cv2.getTrackbarPos("V Max", "HSV Tuner")

        lower_bound = np.array([h_min, s_min, v_min])
        upper_bound = np.array([h_max, s_max, v_max])

        # Convert to HSV and apply mask
        hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
        filtered_output = cv2.bitwise_and(frame, frame, mask=mask)

        # Convert & publish the processed image
        filtered_msg = self.bridge.cv2_to_imgmsg(filtered_output, encoding='bgr8')
        self.hsv_pub.publish(filtered_msg)

        # Display images locally
        cv2.imshow("Original", frame)
        cv2.imshow("Filtered Output", filtered_output)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = HSVTunerNode()
    rclpy.spin(node)
    cv2.destroyAllWind
