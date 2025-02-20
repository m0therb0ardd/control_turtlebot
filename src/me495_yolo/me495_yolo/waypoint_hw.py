#!/usr/bin/env python3
import rclpy    
import math
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_srvs.srv import Empty
from turtle_interfaces.srv import Waypoints
from turtlesim.srv import TeleportAbsolute, Spawn, SetPen
from turtlesim.msg import Pose
from geometry_msgs.msg import Point, Twist
from turtle_interfaces.msg import ErrorMetric



class MyNode(Node):
    def __init__(self):
        super().__init__('waypoint_node')

        ########## Begin ChatGPT Organization Citation ##########
        # declare parameters
        self.declare_parameter('frequency', 1.0)
        self.declare_parameter('tolerance', 1.0)
        self.frequency = self.get_parameter('frequency').value
        self.tolerance = self.get_parameter('tolerance').value

        # initalize variables
        self.waypoints = []
        self.state = 'Stopped' #initial state
        self.complete_loops = 0
        self.total_distance = 0
        self.actual_distance = 0

        # timer
        self.create_timer(0.2/self.frequency, self.timer_callback)

        # log frequency
        self.create_timer(1.0/self.frequency, self.timer_callback)
        self.get_logger().info(f'frequency is {self.frequency}')
        self.get_logger().info(f'tolerance is {self.tolerance}')

        #  create services
        self.callback_group = MutuallyExclusiveCallbackGroup()
        self.complete_loops = 0
        self.total_distance = 0
        self.actual_distance = 0
        self.error_metric_publisher = self.create_publisher(ErrorMetric, 'loop/metrics', 10)

        #create services
        self.toggle_service = self.create_service(Empty, 'toggle' , self.toggle_callback)
        
        #self.waypoint_service = self.create_client(Waypoints, 'load', callback_group=self.callback_group)
        self.load_service = self.create_service(Waypoints, 'load', self.load_callback)
        

        #clients
        # self.waypoint_service = self.create_client(Waypoints, 'load') #fix later !!!!
        # self.waypoint_service = self.create_client(Waypoints, 'load', callback_group=self.callback_group)
        self.teleport_client= self.create_client (TeleportAbsolute, 'turtle1/teleport_absolute',callback_group=self.callback_group)
        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen',callback_group=self.callback_group)
        self.reset_client = self.create_client(Empty, 'reset')

        #wait for services:
        # self.waypoint_service.wait_for_service()
        self.teleport_client.wait_for_service()
        self.pen_client.wait_for_service()
        self.reset_client.wait_for_service()

        #subscribe to pose/position 
        self.pose_subscriber = self.create_subscription(Pose, 'turtle1/pose', self.pose_callback, 10)
        self.current_position = Point()
        self.current_orientation = 0.0


    def toggle_callback(self, request, response):
        if self.state == 'Stopped':
            if not self.waypoints:
                self.get_logger().error('No waypoints have been loaded.')
            else:
                self.state = 'Moving'
                self.current_waypoint_index = 0 
                self.get_logger().info('Starting')
        else:
            self.state = 'Stopped'
            self.get_logger().info("Stopping")
        return response


    async def load_callback(self, request, response):
        self.get_logger().info('entering loadcall back')

        # recieves waypoints (sent by client when service is called)
        self.waypoints = request.waypoints   
        self.total_distance = 0
        self.actual_distance = 0 
        self.complete_loops = 0

        self.reset_turtlesim()
        await self.set_pen_off()

        #distance
        if len(self.waypoints)>1:
            for i in range (len(self.waypoints)-1):
                self.total_distance += self.calculate_distance(self.waypoints[i], self.waypoints[i+1])
        self.get_logger().info(f"total distance traveled to all waypoints: {self.total_distance}")


        for point in self.waypoints: 
           self.get_logger().info('telporting...')
           await self.teleport_to(point.x, point.y)
           
           self.get_logger().info('drawing the X')
           await self.draw_x(point)
        
        if self.waypoints:
            first_waypoint = self.waypoints[0]
            self.get_logger().info('Heading back to first waypoint')
            await self.teleport_to(first_waypoint.x, first_waypoint.y)

        ########## Begin Citation [3] ##########
        await self.set_pen_to_red()
        ########## End Citation [3] ###########


        self.get_logger().info('Completed all waypoints and returned to first')
        self.get_logger().info(f"total distance traveled to all waypoints: {self.total_distance}")

        response.distance = float(self.total_distance)
        return response
    

    # def timer_callback(self):
    #     if self.state == 'Moving':
    #         self.get_logger().info("Issuing Command! ")
    #         self.issue_waypoint_commands()
    #         self.state = 'Stopping'

    #trying new timer_callback using tolerance
    
    ########## Begin Citation [2] ##########
    async def timer_callback(self):
    ########## End Citation [2] ##########
        if self.state == "Moving":
            if self.current_waypoint_index < len(self.waypoints):
                current_target = self.waypoints[self.current_waypoint_index]
                distance = self.calculate_distance(self.current_position, current_target)

                if distance <= self.tolerance:
                    self.current_waypoint_index += 1
                    #await self.set_pen_to_red()

                    if self.current_waypoint_index >= len(self.waypoints):
                        self.complete_loops += 1
                        error = abs(self.total_distance - self.actual_distance)
                        self.publish_error_metrics()
                        self.current_waypoint_index = 0 
                        #await self.set_pen_off()

                else:
                    #await self.set_pen_to_red()
                    self.publish_cmd_vel(current_target)
                    self.actual_distance += 1.0

            else: 
                self.state = 'Stopped' # stop turtle after all points reached

    # turtle movement info
    def publish_error_metrics(self):
        metric_msg = ErrorMetric() # initilizes new instance of this new message type
        metric_msg.complete_loops = self.complete_loops
        metric_msg.actual_distance = self.actual_distance
        metric_msg.error = abs(self.total_distance - self.actual_distance)
        self.error_metric_publisher(metric_msg)


    def publish_cmd_vel(self, target):
        velocity_msg = Twist() #twist is a message type in ROS to describe velocities in linear and angular components
        #first find distance btwn turtle and the waypoint broken into x and y component
        dx = target.x - self.current_position.x
        dy = target.y-self.current_position.y
        distance = math.sqrt(dx**2+dy**2)


        velocity_msg.linear.x = min(0.50, distance)
        angle_to_target = math.atan2(dy, dx)
        velocity_msg.angular.z = angle_to_target- self.current_orientation
        self.velocity_publisher.publish(velocity_msg)
        self.get_logger().info

    ########## Begin Citation [3] ##########
    async def set_pen_off(self):
        request = SetPen.Request()
        request.off = True #pen off
    ########## End Citation [3] ############

        await self.pen_client.call_async(request)

        #future = self.pen_client.call_async(request)
        #rclpy.spin_until_future_complete(self, future)
        #return future #DO NOT DELETE THIS LINE! DO NOT RETURN NONE TYPE WHEN YOU USE AWAIT


    async def set_pen_to_red(self):
        #set_pen_client = self.create_client(SetPen ,'/turtle1/set_pen')
        request = SetPen.Request()
        request.r = int(5)
        request.g = int(186)
        request.b = int(221)
        request.width = int(4)
        request.off = False #pen on 

        await self.pen_client.call_async(request)
        
    ########## Begin Citation [2] ##########
    async def issue_waypoint_commands(self):
    ########## End Citation [2] ##########
    
        self.get_logger().info("Entered issue_waypoint_commands function")  # Debugging

        #The turtle traverses the following waypoints: (1.4, 1.6), (2.2, 9.4), (7.2, 6.1), (4.0, 2.6), (8.2, 1.5), (4.1, 5.3)
        request = Waypoints.Request()

        request.waypoints = [Point(x=1.4, y=1.6), Point(x=2.2, y=9.4), Point(x=7.2, y=6.1), Point(x=4.0, y=2.6), Point(x=8.2, y=2.5), Point(x=4.1, y=5.3)]

        #request.waypoints = waypoints
        await self.load_callback(request=request)


        self.get_logger().info(f"Sending Waypoints: {[f'({p.x}, {p.y})' for p in request.waypoints]}")

        #future = self.waypoint_service.call_async(request)


        
        # Attach a callback to the future instead of blocking
        #future.add_done_callback(self.distance_callback)

        #waypoints_str = [f"({p.x}, {p.y})" for p in request.waypoints]
        #self.get_logger().info(f"Sending Waypoints: {waypoints_str}")


    def distance_callback(self, future):

        try:
            response = future.result()
            if response is not None:
                self.get_logger().info(f"TOTAL DISTANCE: {response.distance}")
                self.state = 'Stopped'
            else:
                self.get_logger().info("Service call returned no response")
        except Exception as e:
            self.get_logger().info(f"Service call failed: {e}")

    def pose_callback(self, msg):
        self.current_position.x = msg.x
        self.current_position.y = msg.y
        self.current_orientation = msg.theta
        #self.get_logger().info(f"Position: {msg.x}, {msg.y}, {msg.y}, orientation: {msg.theta} radians")

    async def teleport_to(self, x, y):
        # if not self.teleport_client.wait_for_service(timeout_sec = 1.0):
        #    self.get_logger().error('Service not available')
        #    return
        self.get_logger().info("Setting pen off...")
        self.get_logger().info("111...")
        # await self.set_pen_off()
        request = TeleportAbsolute.Request()
        request.x = float(x)
        request.y = float(y)
        
        #lets call the service
        self.get_logger().info("Teleporting...")
        await self.teleport_client.call_async(request)


        #use blocking here --> block until service call is complete
        #rclpy.spin_until_future_complete(self, future)

    ########## Begin Citation [2] ##########
    async def reset_turtlesim(self):
    ########## Begin Citation [2] ##########
    
        request = Empty.Request()
        future = self.reset_client.call_async(request)
        #use blocking calls here
        #rclpy.spin_until_future_complete(self, future)
        await future

    async def draw_x(self, point):
        self.get_logger().info("Making pen red...")
        await self.set_pen_to_red()
        self.get_logger().info("pen on now")
        await self.teleport_to(point.x - 0.5, point.y - 0.5)
        await self.teleport_to(point.x + 0.5, point.y + 0.5)
        await self.teleport_to(point.x, point.y)

        await self.teleport_to(point.x - 0.5, point.y + 0.5)
        await self.teleport_to(point.x + 0.5, point.y - 0.5)
        await self.teleport_to(point.x, point.y)
        await self.set_pen_off()

    def calculate_distance(self, point1, point2):
        return math.sqrt((point2.x - point1.x) **2 + (point2.y - point1.y) **2)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__== '__main__':
    main()