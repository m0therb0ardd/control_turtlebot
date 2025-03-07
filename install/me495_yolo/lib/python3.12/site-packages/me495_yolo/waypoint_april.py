''' subscribe to /dancer_position_april
when dancer is in frame record the dancers position for 15 seconds
smooth the path using b spline
convert smoothed waypoints in camera frame (dancer position is already in camera frame so i dont know if i need to do that)
publish the waypoints as markers in rviz 


'''


import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
import tf_transformations
import numpy as np
from scipy.interpolate import splprep, splev
from visualization_msgs.msg import Marker, MarkerArray
import time

class WaypointNodeApril(Node):
    def __init__(self):
        super().__init__('waypoint_generator_april')

        #subscriptions (to april tag position (in camera frame))
        #self.create_subscription(Float32MultiArray, '/dancer_position_april', self.dancer_callback, 10)
        self.create_subscription(Float32MultiArray, '/dancer_position_color', self.dancer_callback, 10)

        #publishers
        self.waypoint_publisher = self.create_publisher(Path, 'dancer_waypoints', 10)
        self.marker_publisher = self.create_publisher(MarkerArray, 'dancer_waypoint_markers', 10)
        self.full_path_publisher = self.create_publisher(Path, 'dancer_full_path', 10)

        # ROS Parameters for B-spline smoothing
        self.declare_parameter('num_waypoints', 5)
        self.declare_parameter('smoothness', 10)

        #storage for dancer path 
        self.dancer_path = []
        
        #timer limit for waypoint processing (i want to only capture 20 seconds of movement)
        self.start_time = None
        self.time_limit = 10
        self.tracking_active = False

    def dancer_callback(self, msg):
        ''' Process dancer position updated from april tag detection and collect dancer points for timer duration '''

        #make sure we have an x and a y coordinate for dancer position (if not report invalid position)
        if len(msg.data)<2:
            self.get_logger().warn("Invalid dancer april tag position data!")
            return 
        
        x, y = msg.data[:2]  # Extract (X, Y) dancer position

        # Start tracking only if first detection
        if self.start_time is None:
            self.start_time = time.time()
            self.tracking_active = True  # Ensure tracking is active
            self.get_logger().info("⏳ Started recording dancer waypoints")

            # 🚨 Stop adding points if tracking is no longer active
        if not self.tracking_active:
            return 

        # Ensure only significant movement is recorded (Threshold = 2cm)
        if len(self.dancer_path) > 0:
            last_x, last_y, _ = self.dancer_path[-1]
            if abs(x - last_x) < 0.02 and abs(y - last_y) < 0.02:  # Ignore tiny movements < 2cm
                return

        #self.get_logger().info(f"💃 Dancer Position Received: X={x:.3f}, Y={y:.3f}")

         # Store dancer path for waypoint generation
        self.dancer_path.append((x, y, 1.0))  # Assume Z=1 (2D navigation)1 is to match depth in rviz for visualization 

        self.publish_full_dancer_path()

        if self.start_time is None:
            self.start_time = time.time()
            self.get_logger().info("⏳ Started recording dancer waypoints")
            self.tracking_active = True


        elapsed_time = time.time() - self.start_time
        #when time limit is reached
        if elapsed_time > self.time_limit:
            #print how many waypoints we are about ot generate 
            num_waypoints = self.get_parameter('num_waypoints').value
            self.get_logger().info(f"📤 Time limit reached! Generating {num_waypoints} waypoints from {len(self.dancer_path)} points.")
            self.tracking_active = False

            #generate those smoothed waypoints
            smoothed_waypoints = self.create_b_spline_waypoints(self.dancer_path)

            #publish those smoothed waypoints
            self.publish_smoothed_waypoints(smoothed_waypoints)

            #reset path and timer for new tracking sequence
            # self.dancer_path = []   
            # self.start_time = None  
            # self.tracking_active = False

    def publish_full_dancer_path(self):
        '''Publish the full dancer path as a continuous path '''
        path_msg = Path()
        path_msg.header.frame_id = "camera_color_optical_frame"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for x, y, z in self.dancer_path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 1.0

            path_msg.poses.append(pose)
        if self.tracking_active == True:
            self.full_path_publisher.publish(path_msg)

    def create_b_spline_waypoints(self, path):
        """generate smooth waypoints using b spline and published smoothed waypoints"""

        num_waypoints = self.get_parameter('num_waypoints').value  # Fetch num_waypoints
        smoothness = self.get_parameter('smoothness').value  # Fetch smoothness

        #need at least 4 waypoints for b spline we should 
        if len(path) < 4:
            self.get_logger().warn("⚠️ Path must have at least 4 points for B-spline.")
            return path

        x = [p[0] for p in path]
        y = [p[1] for p in path]
        z = [p[2] for p in path]

        try:
            tck, _ = splprep([x, y, z], s=smoothness)
            u = np.linspace(0, 1, num_waypoints)
            x_smooth, y_smooth, z_smooth = splev(u, tck)
            #ensure returning floats 
            return list(zip(map(float, x_smooth), map(float, y_smooth), map(float, z_smooth)))
        
        except Exception as e:
            self.get_logger().error(f"🚨 Failed to fit B-spline: {e}")
            return path
        
    def publish_smoothed_waypoints(self, waypoints):
        '''publishes smoothed waypoints in camera frame to /dancer _waypoints as rviz markers'''
        
        #create an empty ros2 path message which is how we will store a series of waypoints as a continuous path 
        path_msg = Path()
        path_msg.header.frame_id = "camera_color_optical_frame"
        path_msg.header.stamp = self.get_clock().now().to_msg() #time stamps path with current ros2 system time

        marker_array = MarkerArray()
        
        #enumerate handles initialization 
        #chat gpt helping me for marker visualization in rviz 
        for i, (x, y, z) in enumerate(waypoints):
            #add each waypoint to path message
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 1.0 # we are dealing with 2d so lets again ensure that z is 1 to streamline visualization in rviz
            path_msg.poses.append(pose)

            #create visualization marker
            marker = Marker()
            marker.header = path_msg.header
            marker.ns = "dancer_waypoints"

            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.05
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.0, 0.0, 1.0
            marker_array.markers.append(marker)

        
        self.waypoint_publisher.publish(path_msg)
        self.marker_publisher.publish(marker_array)
        self.get_logger().info(f"📤 Published {len(waypoints)} smoothed waypoints to /dancer_waypoints")


def main():
    rclpy.init()
    node = WaypointNodeApril()
    rclpy.spin(node)
    rclpy.shutdown()









        


