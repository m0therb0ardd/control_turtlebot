# Dancer-TurtleBot: Tracing Movement in Space

## Overview
The **Dancer-TurtleBot Project** explores the interplay between robotics, movement, and spatial awareness. This project investigates how a TurtleBot3 can interact with a dancer, responding to movement and tracing illuminated pathways that visually represent their shared motion. By leveraging computer vision, real-time tracking, and machine learning, the goal is to create a dynamic, co-evolving relationship between human and robotic movement.

## Project Goals
- Develop a system where a TurtleBot3 tracks a dancer’s movement and responds in real-time.
- Implement a **camera-based tracking** system to follow a designated color (pink object) or AprilTags for precise orientation.
- Generate a **smoothed movement path** using **B-spline algorithms** from tracked motion.
- Create a method for the TurtleBot to **follow the path** and navigate through space effectively.
- Explore movement improvisation principles to shape the robot’s response.
- Investigate **machine learning techniques** to enable adaptive, learned behavior.

## Inspiration
This project draws from:
- **Laban Movement Analysis** to understand movement qualities.
- **Shannon Dowling’s improvisation principles** 
- **Light painting techniques** for visualizing the TurtleBot’s movement.
- Concepts of **home, presence, and spatial awareness** in robotics and dance.

## Implementation
### 1. **Color Tracking Node: Tracking the Dancer and Turtlebot**
- ROS 2 node that tracks objects (dancer pink object, turtlebot center green, turtlbot front orange) using HSV color segmentation and publishes their positions.
- Detects and tracks objects using HSV color filtering.
- Extracts real-world coordinates using camera intrinsics.
- Publishes positions & broadcasts TF frames for tracking.
- Computes TurtleBot’s orientation based on its front and center markers.
- Visualizes detections with OpenCV.

### 2. **Waypoint April Node: Generating a Path**
- The system collects position data of the pink object and and constructs a smooth trajectory using **B-spline interpolation**.
- Subscribes to /dancer_position_color (previously /dancer_position_april) to receive the dancer’s position in the camera frame.
- Records the dancer’s position for 15 seconds when the dancer is first detected.
- Filters insignificant movement (ignores changes smaller than 2 cm).
- Applies B-spline smoothing to the recorded path.
- Publishes smoothed waypoints as a Path message for visualization.
- Displays waypoints in RViz using MarkerArray.
- The final smoothed trajectory is published to /dancer_waypoints.
- 

### 3. **Good Square Node: Navigating the Path**
- Subscribes to /turtlebot_position_color, /turtlebot_orientation_color, and /dancer_waypoints to track the TurtleBot’s position, orientation, and the dancer's movement waypoints in the camera frame.
- Receives and stores waypoints from the dancer’s movement.
- Publishes movement commands (/cmd_vel) to navigate the TurtleBot along the dancer’s path.
- Visualizes waypoints in RViz using Marker messages.
- Checks collinearity between the TurtleBot, its front marker, and the next waypoint to determine movement direction.
- Rotates the TurtleBot if it's not aligned with the waypoint.
- Moves forward when aligned, stopping when it reaches the waypoint.
- Continuously updates waypoints until the path is completed.

This node ensures the TurtleBot follows the dancer’s movement smoothly and accurately, while dynamically adjusting its orientation.

## Setup Instructions
### **Prerequisites**
- **Hardware:** TurtleBot3, Intel RealSense Camera (or equivalent)
- **Software:** ROS 2, OpenCV, YOLOv8/AprilTags, Python, RVIZ2
- **Dependencies:** Install with:
  ```sh
  pip install opencv-python numpy scipy ultralytics
  sudo apt install ros-foxy-apriltag ros-foxy-navigation2
  ```

### **Running the Project**

### 1. Navigate to me495_yolo
In all terminals:
```sh
export ROS_DOMAIN_ID=27
source install/setup.bash
```

### 2. On NUMSR WiFi, SSH into the TurtleBot
```sh
ssh -oSendEnv=30 msr@bebop
export ROS_DOMAIN_ID=27
source install/setup.bash
export TURTLEBOT3_MODEL=burger
```

### 3. Launch the TurtleBot3 Environment
```sh
ros2 launch turtlebot3_bringup robot.launch.py
```

### 4. Launch the RealSense Camera Node
```sh
ros2 launch realsense2_camera rs_launch.py
```

### 5. Launch RViz2
```sh
ros2 run rviz2 rviz2
```

### 6. Run Color Tracking
```sh
ros2 run me495_yolo color_tracking.py
```

### 7. Adjust HSV Values
With color tracking running and markers in sight, run:
```sh
ros2 run me495_yolo hsv_tuner.py
```
Adjust the HSV values in `color_tracking.py` (lines 35-40) as needed.

### 8. Track Pink Object’s Motion and Convert to Waypoints
```sh
ros2 run me495_yolo waypoint_april.py  # (Named poorly; tracks pink object's motion)
```
Ensure the TurtleBot (with green and orange marker) is in view, as this converts the pink path into waypoints and records movement for 15 seconds.

### 9. Send Movement Commands to the TurtleBot
```sh
ros2 run me495_yolo good_square  # (Named poorly; sends velocity commands)
```

### 10. Record and Process Light Trails
Using a separate camera, record the video, then run:
```sh
python3 run me495_yolo light_trail.py  # Overlay light trails onto image
python3 run me495_yolo light_trail_dark.py  # Video of just the light trail
```

