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

        # Subscribe to RGB image, depth image, and camera info
        self.create_subscription(Image, 'image', self.yolo_callback, 10)
        self.create_subscription(Image, '/camera/camera/depth/image_rect_raw', self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10)
        self.create_subscription(MarkerArray, 'waypoint_markers', self.marker_callback, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.freq = 10
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.waypoint_follower = TurtleBotWaypointFollower()


        # Timer variables
        self.start_time = None  # Store start time of movement
        self.time_limit = 8 # Stop recording waypoints after 5 seconds
        self.tf_timer = self.create_timer(0.1, self.broadcast_all_tf)  # Broadcast every 100ms


        self.last_valid_dancer_pose = None  # Store last valid dancer position


        # Publishers
        self.pub = self.create_publisher(Image, 'new_image', 10)
        self.path_publisher = self.create_publisher(Float32MultiArray, 'path_points', 10)
        self.turtlebot_position_pub = self.create_publisher(Float32MultiArray, '/turtlebot_position', 10)
        self.turtlebot_orientation_pub = self.create_publisher(Float32MultiArray, '/turtlebot_orientation', 10)


        # Color Tracking Settings
        self.pink_range = ((145, 50, 50), (165, 255, 255))  # HSV range for pink (dancer)
        #self.pink_range = ((140, 30, 30), (170, 255, 255)) #wider range of pink values

        #self.blue_range = ((35, 50, 50), (85, 255, 255)) #green but not changign naming
        self.blue_range = ((30, 40, 40), (90, 255, 255)) # wider range for more green/blue vlaues --> remember im nto changign name so this is called blue but its green 

        self.yellow_range = ((20, 100, 100), (40, 255, 255))
        #self.blue_range = ((90, 50, 50), (130, 255, 255))   # HSV range for blue (TurtleBot marker)

        # self.color_name = "pink"
        # self.color_range = ((145, 50, 50), (165, 255, 255))  # HSV range for pink

        # # self.color_name = "blue"
        # # self.color_range =  ((90, 50, 50), (130, 255, 255))


        self.depth_image = None  # Store the latest depth frame
        self.fx, self.fy, self.cx, self.cy = None, None, None, None  # Camera intrinsics
        self.pinkavg = np.zeros((int(self.freq*1.0), 2))
        self.blueavg = np.zeros((int(self.freq*1.0), 2)) 
        self.yellowavg = np.zeros((int(self.freq*1.0), 2))

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
            self.get_logger().info(f"✅ Camera Info Received: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}")

    def depth_callback(self, msg):
        """Store the latest depth frame and print a sample value."""
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.get_logger().info(f"✅ Depth Image Received: First Depth Value = {self.depth_image[240, 320]}")  # Sample at center pixel

    def broadcast_all_tf(self):
        """Continuously broadcast the latest known TF frames to keep them in RViz."""
        if hasattr(self, 'latest_blue_world_coords') and self.latest_blue_world_coords:
            Xb, Yb, Zb = self.latest_blue_world_coords
            self.broadcast_camera_to_turtlebot(Xb, Yb, Zb)

        if hasattr(self, 'latest_yellow_world_coords') and self.latest_yellow_world_coords:
            Xy, Yy, Zy = self.latest_yellow_world_coords
            self.broadcast_turtlebot_front_tf(Xb, Yb, Xy, Yy)


    # def yolo_callback(self, image):
    #     """Track pink (dancer), GREEN (TurtleBot), and yellow (TurtleBot front) markers.
    #     - Filters noise using running averages for pink, blue, and yellow.
    #     - Converts centroids to real-world coordinates.
    #     - Computes TurtleBot yaw from blue (base) and yellow (front).
    #     - Publishes data for waypoints, TurtleBot position, and orientation.
    #     """
    #     if self.depth_image is None or self.fx is None:
    #         self.get_logger().warn("Waiting for depth image and camera intrinsics!")
    #         return

    #     # Convert ROS Image to OpenCV
    #     cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
    #     hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    #     ### Step 1: Detect & Process Pink (Dancer)**
    #     pink_cX, pink_cY, pink_world_coords = self.process_color(cv_image, hsv_image, self.pink_range, self.pinkavg, "Dancer", (255, 0, 255))
    #     if pink_world_coords:
    #         Xp, Yp, Zp = pink_world_coords
    #         self.broadcast_camera_to_dancer(Xp, Yp, Zp)
    #         self.track_dancer_path(Xp, Yp, Zp)  # Stores waypoints

    #     # ### Step 2: Detect & Process Blue (TurtleBot)**
    #     # blue_cX, blue_cY, blue_world_coords = self.process_color(cv_image, hsv_image, self.blue_range, self.blueavg, "TurtleBot", (255, 255, 0))

    #     # if blue_world_coords:
    #     #     Xb, Yb, Zb = blue_world_coords
    #     #     self.broadcast_camera_to_turtlebot(Xb, Yb, Zb)  # Stick to this function
    #     #     self.track_turtlebot_position(Xb, Yb)  # Publishes TurtleBot position

    #     # ### Step 3: Detect & Process Yellow (TurtleBot Front)**
    #     # yellow_cX, yellow_cY, yellow_world_coords = self.process_color(cv_image, hsv_image, self.yellow_range, self.yellowavg, "TurtleBot Front", (0, 255, 255))

    #     # if blue_world_coords and yellow_world_coords:
    #     #     self.broadcast_turtlebot_front_tf(Xb, Yb, yellow_world_coords[0], yellow_world_coords[1])

        
    #     # ###  Step 4: Compute TurtleBot Yaw (Orientation)**
    #     # if blue_world_coords and yellow_world_coords:
    #     #     dx = yellow_world_coords[0] - blue_world_coords[0]
    #     #     dy = yellow_world_coords[1] - blue_world_coords[1]
    #     #     yaw = math.atan2(dy, dx)  # Compute yaw angle in radians

    #     #     # Publish TurtleBot orientation
    #     #     orientation_msg = Float32MultiArray()
    #     #     orientation_msg.data = [yaw]
    #     #     self.turtlebot_orientation_pub.publish(orientation_msg)

    #     #     # Broadcast TurtleBot front TF
    #     #     self.broadcast_turtlebot_front_tf(Xb, Yb, yellow_world_coords[0], yellow_world_coords[1])
    #     #     #self.broadcast_turtlebot_tf(Xb, Yb, yaw)
    #     #     self.get_logger().info(f"📏 TurtleBot Yaw: {yaw:.3f} radians")

    #     ### Step 2: Detect & Process Blue (TurtleBot)**
    #     blue_cX, blue_cY, blue_world_coords = self.process_color(cv_image, hsv_image, self.blue_range, self.blueavg, "TurtleBot", (255, 255, 0))

    #     ### Step 3: Detect & Process Yellow (TurtleBot Front)**
    #     yellow_cX, yellow_cY, yellow_world_coords = self.process_color(cv_image, hsv_image, self.yellow_range, self.yellowavg, "TurtleBot Front", (0, 255, 255))

    #     if blue_world_coords:
    #         Xb, Yb, Zb = blue_world_coords
    #         self.broadcast_camera_to_turtlebot(Xb, Yb, Zb)  # ✅ This function exists and is correct

    #         if yellow_world_coords:
    #             Xy, Yy, Zy = yellow_world_coords
    #             self.broadcast_turtlebot_front_tf(Xb, Yb, Zb, Xy, Yy, Zy)  # ✅ Also correct

    #             # ✅ Compute yaw angle
    #             dx = Xy - Xb
    #             dy = Yy - Yb
    #             yaw = math.atan2(dy, dx)  # Compute yaw angle in radians

    #             # ✅ Publish TurtleBot orientation
    #             orientation_msg = Float32MultiArray()
    #             orientation_msg.data = [yaw]
    #             self.turtlebot_orientation_pub.publish(orientation_msg)

    #             self.get_logger().info(f"📏 TurtleBot Yaw: {yaw:.3f} radians")

    #     ### Step 5: Publish Annotated Image**
    #     new_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
    #     self.pub.publish(new_msg)

    # def yolo_callback(self, image):
    #     """Tracks dancer (pink), TurtleBot (blue), and TurtleBot front (yellow).
    #     - Filters noise using running averages.
    #     - Converts centroids to real-world coordinates.
    #     - Computes TurtleBot yaw from blue (base) and yellow (front).
    #     - Publishes TF transforms and orientation.
    #     """
    #     if self.depth_image is None or self.fx is None:
    #         self.get_logger().warn("Waiting for depth image and camera intrinsics!")
    #         return

    #     # Convert ROS Image to OpenCV
    #     cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
    #     hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    #     ### Step 1: Detect & Process Pink (Dancer)
    #     pink_cX, pink_cY, pink_world_coords = self.process_color(
    #         cv_image, hsv_image, self.pink_range, self.pinkavg, "Dancer", (255, 0, 255)
    #     )
    #     if pink_world_coords:
    #         Xp, Yp, Zp = pink_world_coords
    #         self.broadcast_camera_to_dancer(Xp, Yp, Zp)
    #         self.track_dancer_path(Xp, Yp, Zp)  # Stores waypoints

    #     ### Step 2: Detect & Process Blue (TurtleBot)
    #     blue_cX, blue_cY, blue_world_coords = self.process_color(
    #         cv_image, hsv_image, self.blue_range, self.blueavg, "TurtleBot", (255, 255, 0)
    #     )

    #     ### Step 3: Detect & Process Yellow (TurtleBot Front)
    #     yellow_cX, yellow_cY, yellow_world_coords = self.process_color(
    #         cv_image, hsv_image, self.yellow_range, self.yellowavg, "TurtleBot Front", (0, 255, 255)
    #     )

    #     ### Step 4: Broadcast Transforms & Compute Orientation
    #     if blue_world_coords:
    #         Xb, Yb, Zb = blue_world_coords
    #         self.broadcast_camera_to_turtlebot(Xb, Yb, Zb)  # ✅ This function is correct
    #         self.track_turtlebot_position(Xb, Yb, Zb)  # ✅ Ensure position is published

    #         if yellow_world_coords:
    #             Xy, Yy, Zy = yellow_world_coords
    #             self.broadcast_turtlebot_front_tf(Xb, Yb, Zb, Xy, Yy, Zy)  # ✅ Correct frame hierarchy

    #             # ✅ Compute yaw angle (orientation)
    #             dx = Xy - Xb
    #             dy = Yy - Yb
    #             yaw = math.atan2(dy, dx)  # Compute yaw angle in radians

    #             # ✅ Publish TurtleBot orientation
    #             orientation_msg = Float32MultiArray()
    #             orientation_msg.data = [yaw]
    #             self.turtlebot_orientation_pub.publish(orientation_msg)

    #             self.get_logger().info(f"📏 TurtleBot Yaw: {yaw:.3f} radians")

    #     ### Step 5: Publish Annotated Image
    #     new_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
    #     self.pub.publish(new_msg)

    def yolo_callback(self, image):
        """Tracks dancer (pink), TurtleBot (blue), and TurtleBot front (yellow).
        - Filters noise using running averages.
        - Converts centroids to real-world coordinates.
        - Computes TurtleBot yaw from blue (base) and yellow (front).
        - Publishes TF transforms and orientation.
        """
        if self.depth_image is None or self.fx is None:
            self.get_logger().warn("Waiting for depth image and camera intrinsics!")
            return

        # Convert ROS Image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        ### Step 1: Detect & Process Pink (Dancer)
        pink_cX, pink_cY, pink_world_coords = self.process_color(
            cv_image, hsv_image, self.pink_range, self.pinkavg, "Dancer", (255, 0, 255)
        )
        
        if pink_world_coords:
            Xp, Yp, Zp = pink_world_coords
            self.broadcast_camera_to_dancer(Xp, Yp, Zp)
            self.track_dancer_path(Xp, Yp, Zp)  # Stores waypoints

            # ✅ **Start the Timer on First Detection**
            if self.start_time is None:
                self.start_time = time.time()
                self.get_logger().info("⏳ Started tracking dancer path")

            # ✅ **Compute Elapsed Time**
            elapsed_time = time.time() - self.start_time
            self.get_logger().info(f"⏳ Time elapsed: {elapsed_time:.2f}s (Limit: {self.time_limit}s)")

            # ✅ **Publish Path After Time Limit**
            if elapsed_time > self.time_limit and self.dancer_path:
                self.get_logger().info(f"📤 Publishing {len(self.dancer_path)} dancer waypoints!")
                path_msg = Float32MultiArray()
                path_msg.data = [coord for point in self.dancer_path for coord in point]
                self.path_publisher.publish(path_msg)

                # ✅ **Reset for Next Sequence**
                self.dancer_path = []  
                self.start_time = None  # Allows tracking to restart for a new movement sequence

        ### Step 2: Detect & Process Blue (TurtleBot)
        blue_cX, blue_cY, blue_world_coords = self.process_color(
            cv_image, hsv_image, self.blue_range, self.blueavg, "TurtleBot", (255, 255, 0)
        )

        ### Step 3: Detect & Process Yellow (TurtleBot Front)
        yellow_cX, yellow_cY, yellow_world_coords = self.process_color(
            cv_image, hsv_image, self.yellow_range, self.yellowavg, "TurtleBot Front", (0, 255, 255)
        )

        ### Step 4: Broadcast Transforms & Compute Orientation
        if blue_world_coords:
            Xb, Yb, Zb = blue_world_coords
            self.broadcast_camera_to_turtlebot(Xb, Yb, Zb)  # ✅ This function is correct
            self.track_turtlebot_position(Xb, Yb, Zb)  # ✅ Ensure position is published

            if yellow_world_coords:
                
                Xy, Yy, Zy = yellow_world_coords
                self.get_logger().info(f"✅ TurtleBot Front detected! X={Xy:.3f}, Y={Yy:.3f}, Z={Zy:.3f}")
                self.broadcast_turtlebot_front_tf(Xb, Yb, Zb, Xy, Yy, Zy)  # ✅ Correct frame hierarchy

                # ✅ Compute yaw angle (orientation)
                dx = Xy - Xb
                dy = Yy - Yb
                yaw = math.atan2(dy, dx)  # Compute yaw angle in radians
                yaw_degrees = math.degrees(yaw)


                # ✅ Publish TurtleBot orientation
                orientation_msg = Float32MultiArray()
                orientation_msg.data = [yaw_degrees]
                self.turtlebot_orientation_pub.publish(orientation_msg)

                self.get_logger().info(f"📏 TurtleBot Yaw: {yaw:.3f} radians")

        ### Step 5: Publish Annotated Image
        new_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        self.pub.publish(new_msg)



    def process_color(self, cv_image, hsv_image, color_range, avg_filter, label, draw_color):
        """Detects, smooths, converts to world coordinates, and draws a color."""
        mask = cv2.inRange(hsv_image, np.array(color_range[0]), np.array(color_range[1]))
        contour = self.merge_contours(mask)

        if contour is None:
            return None, None, None  # No detection

        # Compute centroid
        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            return None, None, None

        cX = int(moments["m10"] / moments["m00"])
        cY = int(moments["m01"] / moments["m00"])

        # Apply running average filter
        avg_filter[:-1] = avg_filter[1:]  # Shift elements left
        avg_filter[-1] = np.array([cX, cY])  # Insert new value
        cX, cY = np.mean(avg_filter, axis=0).astype(int)  # Compute smoothed centroid

        # Convert to real-world coordinates
        world_coords = self.pixel_to_world(cX, cY)
        if world_coords:
            X, Y, Z = world_coords
            self.get_logger().info(f"🎯 {label} Position: X={X:.3f}, Y={Y:.3f}, Z={Z:.3f}")

        # Draw on image
        cv2.circle(cv_image, (cX, cY), 5, draw_color, -1)
        cv2.putText(cv_image, label, (cX + 10, cY - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, draw_color, 2)

        return cX, cY, world_coords
    
    def track_dancer_path(self, Xp, Yp, Zp):
        """Adds dancer waypoints if movement is significant, publishes path."""
        if len(self.dancer_path) == 0 or (abs(Xp - self.dancer_path[-1][0]) > 0.00 or abs(Yp - self.dancer_path[-1][1]) > 0.05):
            self.dancer_path.append((Xp, Yp, Zp))
            self.get_logger().info(f"📍 Added Dancer Waypoint: ({Xp:.3f}, {Yp:.3f}, {Zp:.3f})")

        elapsed_time = time.time() - self.start_time if self.start_time else 0
        self.get_logger().info(f"⏳ Time elapsed: {elapsed_time:.2f}s (Limit: {self.time_limit}s)")

        if elapsed_time > self.time_limit and self.dancer_path:
            self.get_logger().info(f"📤 Publishing path with {len(self.dancer_path)} points")
            path_msg = Float32MultiArray()
            path_msg.data = [coord for point in self.dancer_path for coord in point]
            self.path_publisher.publish(path_msg)
            self.dancer_path = []  # Reset path


    # def track_turtlebot_position(self, Xb, Yb):
    #     """Publishes TurtleBot's smoothed position."""
    #     turtlebot_msg = Float32MultiArray()
    #     turtlebot_msg.data = [Xb, Yb]  # Only X, Y since it moves on a plane
    #     self.turtlebot_position_pub.publish(turtlebot_msg)

    #     # Update TurtleBot position for waypoint following
    #     self.waypoint_follower.update_turtlebot_position(Xb, Yb)

    def track_turtlebot_position(self, Xb, Yb, Zb):
        """Publishes TurtleBot's smoothed position and ensures TF updates even when stationary."""
        turtlebot_msg = Float32MultiArray()
        turtlebot_msg.data = [Xb, Yb, Zb]  
        self.turtlebot_position_pub.publish(turtlebot_msg)

        # Update TurtleBot position for waypoint following
        self.waypoint_follower.update_turtlebot_position(Xb, Yb)

        # Always update latest known TurtleBot position
        self.latest_blue_world_coords = (Xb, Yb, Zb)

        # **Ensure TF updates even when stationary**
        self.broadcast_camera_to_turtlebot(Xb, Yb, Zb)



    def merge_contours(self, mask):
        """Find and merge nearby contours to avoid gaps in tracking."""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        merged_contour = None

        for contour in contours:
            if cv2.contourArea(contour) > 300:  # Filter out small areas
                if merged_contour is None:
                    merged_contour = contour
                else:
                    merged_contour = np.vstack((merged_contour, contour))

        return merged_contour



    def draw_paths(self, image):
        """Draw green paths for the dancer and TurtleBot."""
        
        # Draw the dancer's path
        if len(self.dancer_path) > 1:
            for i in range(1, len(self.dancer_path)):
                cv2.line(image, self.dancer_path[i - 1], self.dancer_path[i], (0, 255, 0), 2)

        # # Draw the TurtleBot's path
        # if len(self.turtlebot_path) > 1:
        #     for i in range(1, len(self.turtlebot_path)):
        #         cv2.line(image, self.turtlebot_path[i - 1], self.turtlebot_path[i], (0, 255, 0), 2)



    def find_contour_center(self, mask):
        """Find the largest contour in the mask and return its centroid (cX, cY)."""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            moments = cv2.moments(largest_contour)
            if moments["m00"] != 0:
                cX = int(moments["m10"] / moments["m00"])
                cY = int(moments["m01"] / moments["m00"])
                return cX, cY
        return None, None

    def image_to_pixel(self, world_point):
        """Convert real-world (X, Y, Z) to pixel coordinates for visualization."""
        x, y, z = world_point
        u = int((x * self.fx / z) + self.cx)
        v = int((y * self.fy / z) + self.cy)
        return (u, v)
    

    def pixel_to_world(self, u, v):
        """Convert pixel coordinates (u, v) and depth to real-world coordinates."""
        if self.depth_image is None or self.fx is None:
            return None  # Wait until we have depth and camera intrinsics

        # Ensure pixel coordinates are within valid image bounds
        h, w = self.depth_image.shape  # Get depth image size
        u = np.clip(u, 0, w - 1)  # Clamp x-coordinate
        v = np.clip(v, 0, h - 1)  # Clamp y-coordinate

        # Get depth at the pixel
        depth = self.depth_image[v, u] * 0.001  # Convert mm to meters
        if depth <= 0:  # Invalid depth
            return None

        # Convert to real-world coordinates using intrinsics
        X = (u - self.cx) * depth / self.fx
        Y = (v - self.cy) * depth / self.fy
        Z = depth  # Depth is the Z coordinate


        self.get_logger().info(f" Dancer in Camera Frame: X={X:.3f}, Y={Y:.3f}, Z={Z:.3f}")
        self.get_logger().info(f"Depth at ({u},{v}) = {depth:.3f} meters")


        return X, Y, Z
    
    def broadcast_camera_to_dancer(self, X, Y, Z):
        """Broadcast transformation from camera frame to detected pink object (dancer)."""
        try:
            t = TransformStamped()

            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'camera_color_optical_frame'  # Camera's frame
            t.child_frame_id = 'dancer_pink_object'  # New frame for dancer

            t.transform.translation.x = float(X)
            t.transform.translation.y = float(Y)
            t.transform.translation.z = float(Z)

            # Set rotation to identity quaternion (no rotation assumed)
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0

            # Broadcast transformation
            self.tf_broadcaster.sendTransform(t)
        except Exception as e:
            self.get_logger().error(f"Failed to publish TF transform: {e}")

    def broadcast_camera_to_turtlebot(self, X, Y, Z):
        """Broadcast transformation from camera frame to detected blue object (TurtleBot)."""
        try:
            self.get_logger().info(f"📡 Broadcasting TurtleBot TF: X={X:.3f}, Y={Y:.3f}, Z={Z:.3f}")

            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'camera_color_optical_frame'  # Camera's frame
            t.child_frame_id = 'turtlebot_blue_object'  # New frame for TurtleBot marker

            t.transform.translation.x = float(X)
            t.transform.translation.y = float(Y)
            t.transform.translation.z = float(Z)

            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0

            self.tf_broadcaster.sendTransform(t)
            self.get_logger().info(f"✅ Successfully sent TF for TurtleBot!")

        except Exception as e:
            self.get_logger().error(f"❌ Failed to publish TF transform for TurtleBot: {e}")


    # def broadcast_turtlebot_tf(self, Xb, Yb, yaw):
    #     """Broadcast transformation for TurtleBot including orientation."""
    #     try:
    #         t = TransformStamped()
    #         t.header.stamp = self.get_clock().now().to_msg()
    #         t.header.frame_id = 'camera_color_optical_frame'  # Reference frame
    #         t.child_frame_id = 'turtlebot_blue_object'  # TurtleBot frame

    #         # Position
    #         t.transform.translation.x = float(Xb)
    #         t.transform.translation.y = float(Yb)
    #         t.transform.translation.z = 0.0  # Assume TurtleBot is on a flat plane

    #         # Convert yaw to quaternion
    #         quat = tf_transformations.quaternion_from_euler(0, 0, yaw)
    #         t.transform.rotation.x = quat[0]
    #         t.transform.rotation.y = quat[1]
    #         t.transform.rotation.z = quat[2]
    #         t.transform.rotation.w = quat[3]

    #         # Broadcast transformation
    #         self.tf_broadcaster.sendTransform(t)
    #         self.get_logger().info(f"📡 Broadcasted TurtleBot TF: X={Xb:.3f}, Y={Yb:.3f}, Yaw={yaw:.3f} rad")

    #     except Exception as e:
    #         self.get_logger().error(f"Failed to publish TF transform for TurtleBot: {e}")


    # def broadcast_turtlebot_front_tf(self, Xb, Yb, Zb, Xy, Yy, Zy):
    #     """Broadcast transformation from TurtleBot's base to its front."""
    #     try:
    #         t = TransformStamped()
    #         t.header.stamp = self.get_clock().now().to_msg()
    #         t.header.frame_id = 'turtlebot_blue_object'  # Base of TurtleBot
    #         t.child_frame_id = 'turtlebot_front'  # New frame for front

    #         # ✅ Set the correct position relative to the TurtleBot base
    #         t.transform.translation.x = float(Xy - Xb)
    #         t.transform.translation.y = float(Yy - Yb)
    #         t.transform.translation.z = float(Zy - Zb)

    #         # ✅ Compute rotation from yaw (Z-axis rotation)
    #         yaw = math.atan2(Yy - Yb, Xy - Xb)
    #         quat = tf_transformations.quaternion_from_euler(0, 0, yaw)
    #         t.transform.rotation.x = quat[0]
    #         t.transform.rotation.y = quat[1]
    #         t.transform.rotation.z = quat[2]
    #         t.transform.rotation.w = quat[3]

    #         # ✅ Broadcast transformation
    #         self.tf_broadcaster.sendTransform(t)
    #         self.get_logger().info(f"📡 Broadcasted TurtleBot Front TF: X={Xy:.3f}, Y={Yy:.3f}, Z={Zy:.3f}, Yaw={yaw:.3f} rad")

    #     except Exception as e:
    #         self.get_logger().error(f"Failed to publish TF transform for TurtleBot front: {e}")

    # def broadcast_turtlebot_front_tf(self, Xb, Yb, Zb, Xy, Yy, Zy):
    #     """Broadcast transformation from TurtleBot's base to its front marker."""
    #     try:
    #         self.get_logger().info(f"📡 Broadcasting TurtleBot Front TF: Parent (Blue): X={Xb:.3f}, Y={Yb:.3f}, Z={Zb:.3f} | Front (Yellow): X={Xy:.3f}, Y={Yy:.3f}, Z={Zy:.3f}")

    #         t = TransformStamped()
    #         t.header.stamp = self.get_clock().now().to_msg()
    #         t.header.frame_id = 'turtlebot_blue_object'  # ✅ Parent frame
    #         t.child_frame_id = 'turtlebot_front'  # ✅ Child frame

    #         # ✅ Relative translation from TurtleBot base to front marker
    #         t.transform.translation.x = float(Xy - Xb)
    #         t.transform.translation.y = float(Yy - Yb)
    #         t.transform.translation.z = float(Zy - Zb)  # Keep Z difference

    #         # ✅ Compute yaw angle (orientation)
    #         yaw = math.atan2(Yy - Yb, Xy - Xb)
    #         quat = tf_transformations.quaternion_from_euler(0, 0, yaw)
    #         t.transform.rotation.x = quat[0]
    #         t.transform.rotation.y = quat[1]
    #         t.transform.rotation.z = quat[2]
    #         t.transform.rotation.w = quat[3]

    #         self.tf_broadcaster.sendTransform(t)
    #         self.get_logger().info(f"✅ Successfully Sent TurtleBot Front TF! Yaw: {yaw:.3f} rad ({math.degrees(yaw):.1f}°)")

    #     except Exception as e:
    #         self.get_logger().error(f"❌ Failed to publish TF transform for TurtleBot Front: {e}")


    def broadcast_turtlebot_front_tf(self, Xb, Yb, Zb, Xy, Yy, Zy):
        """Broadcast transformation from TurtleBot base to its front marker, ensuring stability."""
        try:
            # ✅ Use last known TurtleBot position if flickering
            if hasattr(self, 'last_valid_turtlebot_position'):
                if abs(Xb - self.last_valid_turtlebot_position[0]) > 0.1 or \
                abs(Yb - self.last_valid_turtlebot_position[1]) > 0.1:
                    self.get_logger().warn("⚠️ Large jump detected in TurtleBot position! Using last known position.")
                    Xb, Yb, Zb = self.last_valid_turtlebot_position
            
            # ✅ Store last valid TurtleBot position
            self.last_valid_turtlebot_position = (Xb, Yb, Zb)

            self.get_logger().info(f"📡 Broadcasting TurtleBot Front TF: Parent: (Xb={Xb:.3f}, Yb={Yb:.3f}, Zb={Zb:.3f}) | Front: (Xy={Xy:.3f}, Yy={Yy:.3f}, Zy={Zy:.3f})")

            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'turtlebot_blue_object'  # Base of TurtleBot
            t.child_frame_id = 'turtlebot_front'  # New frame for front marker

            # ✅ Ensure relative transformation is correct
            t.transform.translation.x = float(Xy - Xb)
            t.transform.translation.y = float(Yy - Yb)
            t.transform.translation.z = float(Zy - Zb)

            # ✅ Compute yaw from TurtleBot base to front
            yaw = math.atan2(Yy - Yb, Xy - Xb)
            quat = tf_transformations.quaternion_from_euler(0, 0, yaw)
            t.transform.rotation.x = quat[0]
            t.transform.rotation.y = quat[1]
            t.transform.rotation.z = quat[2]
            t.transform.rotation.w = quat[3]

            self.tf_broadcaster.sendTransform(t)
            self.get_logger().info(f"✅ Successfully Broadcasted TurtleBot Front TF! Yaw={yaw:.3f} rad ({math.degrees(yaw):.1f}°)")

        except Exception as e:
            self.get_logger().error(f"❌ Failed to publish TF transform for TurtleBot Front: {e}")


    def get_turtlebot_position(self):
        """Get the real-time TurtleBot position from TF."""
        try:
            transform = self.tf_buffer.lookup_transform('map', 'turtlebot_base', rclpy.time.Time())
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z
            self.get_logger().info(f"✅ Real TurtleBot Position: X={x}, Y={y}")
            return x, y, z
        except Exception as e:
            self.get_logger().warn(f"⚠️ Failed to get TurtleBot position: {e}")
            return None

    def compute_dancer_in_turtlebot_frame(self):
        """Compute dancer’s position in the TurtleBot’s reference frame using TF."""
        try:
            # Transform dancer position into TurtleBot's frame
            #transform = self.tf_buffer.lookup_transform('turtlebot_base', 'dancer_pink_object', rclpy.time.Time())
            transform = self.tf_buffer.lookup_transform('turtlebot_blue_object', 'dancer_pink_object', rclpy.time.Time())

            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z

            self.last_valid_dancer_pose = (x, y, z)
            self.get_logger().info(f"✅ Dancer in TurtleBot Frame: X={x:.3f}, Y={y:.3f}, Z={z:.3f}")

            return x, y, z
        except Exception as e:
            self.get_logger().warn(f"⚠️ Failed to compute dancer in TurtleBot frame: {e}")
            return self.last_valid_dancer_pose if self.last_valid_dancer_pose else None

    
    def get_camera_to_turtlebot_transform(self):
        """Get real-time transform from camera to TurtleBot."""
        try:
            transform = self.tf_buffer.lookup_transform('turtlebot_base', 'camera_color_optical_frame', rclpy.time.Time())
            self.get_logger().info(f"✅ Real Camera->TurtleBot Transform: X={transform.transform.translation.x}, Y={transform.transform.translation.y}")
            return transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z
        except Exception as e:
            self.get_logger().warn(f"⚠️ Failed to get camera-to-TurtleBot transform: {e}")
            return None




    def marker_callback(self, msg):
        """Receive waypoint markers for visualization.""" 
        self.get_logger().info(f"Received {len(msg.markers)} markers")
        self.waypoint_markers = msg.markers

def main():
    rclpy.init()
    node = YoloNode()
    rclpy.spin(node)
    rclpy.shutdown()





    # def yolo_callback(self, image):
    #     """vTrack pink (dancer) and blue (TurtleBot) markers, merge contours, update paths, and publish real-world coordinates."""
    #     """ Finds pink countour, filters noise using running average, converts centroid to real world coords using pixel_to_world, tracks drancers path and punlishes waypoints"""
    #     """" for yellow detection (turtlebot front) computes vector dx dyt from blue to yellow, uses atan2 to calc yaw, published yaw to /turtlebot_orientation """
    #     elapsed_time = 0  # ✅ Ensure elapsed_time is always defined at the start

    #     if self.depth_image is None or self.fx is None:
    #         self.get_logger().warn("Waiting for depth image and camera intrinsics!")
    #         return

    #     # Convert ROS Image to OpenCV
    #     cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
    #     hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    #     ### **Step 1: Detect & Merge Pink Contours (Dancer)**
    #     pink_mask = cv2.inRange(hsv_image, np.array(self.pink_range[0]), np.array(self.pink_range[1]))
    #     pink_contour = self.merge_contours(pink_mask)

    #     if pink_contour is not None:
    #         moments = cv2.moments(pink_contour)
    #         if moments["m00"] != 0:
    #             curr_pink_cX = int(moments["m10"] / moments["m00"])
    #             curr_pink_cY = int(moments["m01"] / moments["m00"])

    #             if (curr_pink_cX !=0 and curr_pink_cY !=0):
    #                 # Running average: Shift all elements left
    #                 self.pinkavg[:-1] = self.pinkavg[1:]

    #                 # Insert new position at the last index
    #                 self.pinkavg[-1] = np.array([curr_pink_cX, curr_pink_cY])

    #             # Compute smoothed centroid
    #             pink_cX, pink_cY = np.mean(self.pinkavg, axis=0)

    #             pink_cX = int(pink_cX)
    #             pink_cY = int(pink_cY) 

    #             ### **Step 2: Convert Pink Centroid to Real-World Coordinates**
    #             pink_world_coords = self.pixel_to_world(pink_cX, pink_cY)

    #             if pink_world_coords:
    #                 Xp, Yp, Zp = pink_world_coords
    #                 self.broadcast_camera_to_dancer(Xp, Yp, Zp)
    #                 self.get_logger().info(f"🩷 Dancer Position: X={Xp:.3f}, Y={Yp:.3f}, Z={Zp:.3f}")

    #                 ### ✅ **Step 3: Start Timer on First Detection**
    #                 if self.start_time is None:
    #                     self.start_time = time.time()  # Start tracking when dancer is first detected
    #                     self.get_logger().info("⏳ Started tracking dancer path")

    #                 # ✅ Always define elapsed_time
    #                 elapsed_time = time.time() - self.start_time
    #                 self.get_logger().info(f"⏳ Timer Check: {elapsed_time:.2f}s elapsed")

    #                 ### **Step 5: Add to Path Only if Movement is Significant**
    #                 if len(self.dancer_path) == 0 or (
    #                     abs(Xp - self.dancer_path[-1][0]) > 0.00 or abs(Yp - self.dancer_path[-1][1]) > 0.05 ## MODIFIED THREHSOLDDDDD
    #                 ):
    #                     self.dancer_path.append((Xp, Yp, Zp))  
    #                     self.get_logger().info(f"📍 Added Waypoint: ({Xp:.3f}, {Yp:.3f}, {Zp:.3f})")

    #                 ### **Step 6: Draw Pink Centroid**
    #                 cv2.circle(cv_image, (pink_cX, pink_cY), 5, (255, 0, 255), -1)
    #                 cv2.putText(cv_image, "Dancer", (pink_cX + 10, pink_cY - 10),
    #                             cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)

    #     ### **Step 7: Publish the Final Waypoints Once After 5 Seconds**
    #     if elapsed_time > self.time_limit and len(self.dancer_path) > 0:
    #         path_msg = Float32MultiArray()
    #         path_msg.data = [coord for point in self.dancer_path for coord in point]
    #         self.get_logger().info(f"📤 Publishing Final 5-Second Dancer Path: {path_msg.data}")
    #         self.path_publisher.publish(path_msg)

    #         # **Clear the dancer path to prevent further updates in RViz**
    #         self.dancer_path = []  

    #     ### **Step 8: Publish the Annotated Image**
    #     new_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
    #     self.pub.publish(new_msg)


    #     ### **Step 2: Detect & Merge Blue Contours (TurtleBot)**
    #     blue_mask = cv2.inRange(hsv_image, np.array(self.blue_range[0]), np.array(self.blue_range[1]))
    #     blue_contour = self.merge_contours(blue_mask)

    #     if blue_contour is not None:
    #         moments = cv2.moments(blue_contour)
    #         if moments["m00"] != 0:
    #             curr_blue_cX = int(moments["m10"] / moments["m00"])
    #             curr_blue_cY = int(moments["m01"] / moments["m00"])

    #             if (curr_blue_cX != 0 and curr_blue_cY != 0):
    #                 # Running average: Shift all elements left
    #                 self.blueavg[:-1] = self.blueavg[1:]

    #                 # Insert new position at the last index
    #                 self.blueavg[-1] = np.array([curr_blue_cX, curr_blue_cY])

    #             # Compute smoothed centroid
    #             blue_cX, blue_cY = np.mean(self.blueavg, axis=0)

    #             blue_cX = int(blue_cX)
    #             blue_cY = int(blue_cY)

    #             ### **Step 3: Convert Blue Centroid to Real-World Coordinates**
    #             blue_world_coords = self.pixel_to_world(blue_cX, blue_cY)

    #             if blue_world_coords:
    #                 Xb, Yb, Zb = blue_world_coords
    #                 self.broadcast_camera_to_turtlebot(Xb, Yb, Zb)
    #                 self.get_logger().info(f"🔵 TurtleBot Position: X={Xb:.3f}, Y={Yb:.3f}, Z={Zb:.3f}")

    #                 ### **Step 4: Only Add to Path if Movement is Significant**
    #                 if len(self.turtlebot_path) == 0 or (
    #                     abs(Xb - self.turtlebot_path[-1][0]) > 0.05 or abs(Yb - self.turtlebot_path[-1][1]) > 0.05
    #                 ):
    #                     self.turtlebot_path.append((Xb, Yb, Zb))  
    #                     self.get_logger().info(f"📍 Added TurtleBot Waypoint: ({Xb:.3f}, {Yb:.3f}, {Zb:.3f})")

    #                 ### **Step 5: Publish Smoothed TurtleBot Position**
    #                 turtlebot_msg = Float32MultiArray()
    #                 turtlebot_msg.data = [Xb, Yb]  # Only X, Y since TurtleBot moves on a plane
    #                 self.turtlebot_position_pub.publish(turtlebot_msg)

    #                 # Update TurtleBot position in the waypoint follower
    #                 self.waypoint_follower.update_turtlebot_position(Xb, Yb)

    #                 ### **Step 6: Draw Blue Centroid**
    #                 cv2.circle(cv_image, (blue_cX, blue_cY), 5, (255, 255, 0), -1)
    #                 cv2.putText(cv_image, "TurtleBot", (blue_cX + 10, blue_cY - 10),
    #                             cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)


    #     ### **Step 3: Detect & Merge Yellow Contours (TurtleBot Front)**
    #     yellow_mask = cv2.inRange(hsv_image, np.array(self.yellow_range[0]), np.array(self.yellow_range[1]))
    #     yellow_contour = self.merge_contours(yellow_mask)

    #     yellow_world_coords = None
    #     if yellow_contour is not None:
    #         moments = cv2.moments(yellow_contour)
    #         if moments["m00"] != 0:
    #             curr_yellow_cX = int(moments["m10"] / moments["m00"])
    #             curr_yellow_cY = int(moments["m01"] / moments["m00"])

    #             # Compute smoothed centroid
    #             yellow_cX, yellow_cY = int(curr_yellow_cX), int(curr_yellow_cY)

    #             # Convert to world coordinates
    #             yellow_world_coords = self.pixel_to_world(yellow_cX, yellow_cY)
    #             if yellow_world_coords:
    #                 Xy, Yy, Zy = yellow_world_coords
    #                 self.get_logger().info(f"🟡 TurtleBot Front Position: X={Xy:.3f}, Y={Yy:.3f}, Z={Zy:.3f}")

    #                 # Draw Yellow Marker
    #                 cv2.circle(cv_image, (yellow_cX, yellow_cY), 5, (0, 255, 255), -1)
    #                 cv2.putText(cv_image, "TurtleBot Front", (yellow_cX + 10, yellow_cY - 10),
    #                             cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    #     ### **Step 4: Compute Orientation (Yaw)**
    #     if blue_world_coords and yellow_world_coords:
    #         dx = Xy - Xb
    #         dy = Yy - Yb
    #         yaw = math.atan2(dy, dx)  # Compute yaw angle in radians

    #         # Publish orientation
    #         orientation_msg = Float32MultiArray()
    #         orientation_msg.data = [yaw]
    #         self.turtlebot_orientation_pub.publish(orientation_msg)

    #         Xb, Yb, _ = blue_world_coords
    #         Xy, Yy, _ = yellow_world_coords

    #         # Broadcast the front TF
    #         self.broadcast_turtlebot_front_tf(Xb, Yb, Xy, Yy)

    #         self.get_logger().info(f"📏 TurtleBot Yaw (Orientation): {yaw:.3f} radians")

    #         # Broadcast TF with orientation
    #         self.broadcast_turtlebot_tf(Xb, Yb, yaw)


    #     ### **Step 3: Publish the Annotated Image**
    #     new_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
    #     self.pub.publish(new_msg)