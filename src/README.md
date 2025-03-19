# Project: Symbiotic Dance Between a TurtleBot3 and a Dancer  
*Work in Progress*

## Project Idea  
This project explores the interplay between spatial awareness, the concept of home, and human-robot collaboration. The TurtleBot3 and a dancer create illuminated pathways that visually trace their movements, engaging in a creative dialogue about presence and belonging. Machine learning analyzes these light-drawn patterns, enabling dynamic responses between the robot and human. Inspired by Shannon Dowling's improvisational principles and Laban Dance Movement Analysis, this project combines robotics and performance art.

---

## Plan  
The project is structured into the following key stages:

1. **TurtleBot Setup and Basic Functionality**  
   - Ensure both TurtleBots are operational and capable of executing basic movement commands (forward, turn, stop).  
   - Develop initial prompts to facilitate movement and interaction.

2. **Orientation and Navigation**  
   - Implement north orientation using IMU and magnetometer data.  
   - Utilize LiDAR to navigate toward open spaces and avoid obstacles.

3. **Lighting Display and Interaction**  
   - Create an illuminated display using Arduino-controlled lights to visually trace the TurtleBot’s movements.  
   - Experiment with dynamic lighting patterns to enhance the artistic component of the performance.

4. **Path and Color Detection**  
   - Use a RealSense camera or OptiTrack system to detect paths and colors associated with light trails.  
   - Develop algorithms to enable the TurtleBot to follow light-based paths.

5. **Path Following and Shape Recognition**  
   - Integrate path-following behavior into the TurtleBot’s navigation system.  
   - Incorporate machine learning to train the robot for recognizing and responding to specific shapes or light patterns.

6. **Testing and Refinement**  
   - Conduct extensive testing of the system in a creative space, ensuring smooth interaction between the TurtleBot and dancer.  
   - Refine the choreography and technical setup to achieve a cohesive performance.


---

## Progress  

### Week 1: (January 6, 2025)
- Verified TurtleBot functionality for both robots (Teetle and Shredder).
- Created a GitHub repository to document project development.
- Programmed basic movement commands: forward, turn, and stop.
- Finalized the project proposal.
- Added a magnetic sensor to calculate north orientation.
- Began programming north orientation using the IMU (in progress).

### Week 2: (January 13, 2025)
- Improved north orientation functionality.
  - Filtered non-zero values in the `orient_to_north` node for better accuracy.
  - Noted that magnetometer values were inconsistent; identified the need for calibration without interference (e.g., on a non-metallic surface).
- Ordered a new magnetometer for reliable north orientation calculations.
- Prepared code for `orient_to_north` and `orient_to_home` to deploy once the magnetometer is operational.
- Started implementing and testing LiDAR-based movement towards open spaces.
- Began exploring path detection for dynamic navigation.
- Examined the [Light Painting Robot GitHub repository](https://github.com/241abhishek/Light-Painting-Robot/tree/main) for inspiration and technical insights.
- Added realsense to turtlebot (TEETLE) following these commands: : Manually Build the RealSense SDK From Source 
sudo apt update
sudo apt install git cmake build-essential libssl-dev libusb-1.0-0-dev pkg-config
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig
realsense-viewer


### Week 3: (January 20, 2025) [shifting to daily updates]
- NEED TO GET ORIENT TO NORTH TO WORK (SET UP MAGNETOMETER) --> I could use apirl tag but i want it do be dark so no
- MOVE TOWARDS EMPTY SPACE AND STOP (LIDAR WORKS BUT MAYBE I SHOULD USE REALSENSE CAMERA ISNTEAD)--> ask about camera usage
- 3D PRINT SOMETHING TO AFFIX REALSENSE CAMERA TO TURTLEBOT
- ORIENT TOWARDS DANCER (TOWARDS BLUE LIGHT) AND MOVE FORWARD

#### Monday Jan 20 
 Efforts to Get RealSense Working

Installed and Tested RealSense ROS 2 Package:
 - Installed realsense2_camera for ROS 2.
 - Launched the node using:

ros2 launch realsense2_camera rs_launch.py

Checked published topics using:

- ros2 topic list

- Verified active topics like /camera/camera/color/image_raw and /camera/camera/depth/image_rect_raw.

Troubleshooted RealSense Errors:

- Encountered and debugged errors like:
        xioctl(UVCIOC_CTRL_QUERY) failed (protocol error).
        Issues with RealSense not being detected by the node.
- Verified USB device detection with:

lsusb

Verified kernel logs for USB-related issues using:

    dmesg | grep -i usb

Explored UDEV Rules and Permissions:

- Added UDEV rules for RealSense to resolve access permissions.
    Reloaded UDEV rules:

    sudo udevadm control --reload-rules && sudo udevadm trigger

Lowered Camera Resolution and Frame Rate:

- Attempted to reduce data throughput by customizing RealSense launch parameters:

        ros2 launch realsense2_camera rs_launch.py depth_width:=640 depth_height:=480 depth_fps:=15

- Attempted to Visualize Data in RViz:
    - Added an Image Display in RViz and set the topic to /camera/camera/color/image_raw.
        Adjusted QoS settings in RViz to Best Effort.

Efforts to Get USB Camera Working

    Installed ROS USB Camera Packages:
        Installed and tested v4l2_camera and usb_cam packages:

    sudo apt install ros-iron-v4l2-camera
    sudo apt install ros-iron-usb-cam

Launched USB Camera Nodes:

    Successfully started the v4l2_camera node on the TurtleBot:

ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/video0

Verified the camera's device path (/dev/video0) and properties using:

    v4l2-ctl --list-devices

Checked Camera Topics:

    Confirmed the publication of /image_raw and /camera_info topics:

    ros2 topic list

Visualized USB Camera Data in RViz:

    Added an Image Display in RViz with the topic /image_raw.
    Adjusted QoS settings to Best Effort.

Attempted Debugging with RQt Image Viewer:

    Tested rqt_image_view as an alternative visualization tool:

sudo apt install ros-iron-rqt-image-view
rqt_image_view

### Plan to Pivot:
 - camera from above and use yolov8 to identify orientatin of the turtle bot diffferent colored light on each face of the turtlebot. 
 - for example when white light on turtle bot is aligned with blue dot of dancer stop (turtlebot faces dancer )
 - RealSense Setup on Your Laptop:

    Use the RealSense camera to capture video streams.
    Process these streams on your laptop.

- YOLOv8 Object Detection:

    Train or fine-tune YOLOv8 to recognize specific markers (e.g., colored lights on the TurtleBot and a blue dot for the dancer).
    Use YOLOv8's bounding box output to determine the relative orientation of the TurtleBot.

- Control Logic:

    Implement logic to stop the TurtleBot when its white light aligns with the dancer’s blue dot.

- Communication with TurtleBot:

    Use ROS to send movement commands to the TurtleBot.



### Tuesday Jan 21
Sensor Setup:

- Connected the QMC5883L magnetometer to the TurtleBot's Raspberry Pi.
- Verified the sensor connection using i2cdetect, which confirmed the device is detected at address 0x0D.

Testing Raw Data:

- Ran scripts to fetch raw X, Y, Z data from the sensor.
- Observed that the raw data changes with rotation, but the values did not align logically with expected cardinal directions.

Initial Heading Calculation:

- Wrote a Python script to calculate heading using math.atan2(y, x) based on raw X, Y data.
- Observed heading values for north, east, south, and west but found them inconsistent with true cardinal directions.

Magnetic Declination:

- Attempted to incorporate the magnetic declination for your location (4° 7' W, ~-4.1167°).
- Adjusted heading calculation but still encountered discrepancies.

Calibration Attempts:

- Implemented a calibration step to calculate X and Y offsets based on maximum and minimum raw values during a 360° rotation.
- Applied offsets to normalize raw data but observed that the heading values remained inaccurate.

Exploration of External Libraries:

- Tried using the py-qmc5883l Python library to simplify sensor interaction.
 - Installed the library on the TurtleBot and local virtual environment but encountered:
 - Dependency issues (smbus2 was missing initially).
- Method issues (get_bearing not available in the library).

Raw Data Observations:

- Verified raw X, Y, Z values for each cardinal direction:
        North: X: 2080, Y: 4137, Z: 2328
        East: X: 2092, Y: 4411, Z: 2338
        South: X: 2137, Y: 2898, Z: 2237
        West: X: 1963, Y: 2442, Z: 2208
- Identified minimal changes in X values and significant changes in Y values, suggesting improper alignment, calibration, or interference.

## Reground in Visual Components Momentarily 
- realsense from above is going to track my dancer and turtlebot
- teetle will have a differnt colored light for each of four sides to track orientation 
- dancer will have a head light two arm lights and two leg lights 
- i can have teetle follow one path head path and then left arm path and then right leg path and layer the videos 


### Wednesday Jan 22 
- Setup YOLO in ROS:

    Installed the ultralytics package and set up a ROS workspace (~/yolo_home) for running YOLO.
    Built and sourced the workspace to make the YOLO node functional.

- Topic Remapping:

    Adjusted the YOLO node to subscribe to the RealSense image topic by running: ros2 run me495_yolo yolo --ros-args -r /image:=/camera/camera/color/image_raw

- Color Tracking Implementation: 
    - Modified the YOLO node to detect specific colors (pink, orange, white, green, blue, yellow).
    -  Used HSV filtering for precise color segmentation.
   -  Drew bounding boxes and labeled detected colors in the output image.


### Thursday Jan 23
- tracked color from centroid
- created path of color 
- trying smoothing path with moving average filter
- smoothing filters didnt quite work the way i wanted so instead i merged contours --> pushkar says i can do smoothing in waypoint 
- now i set timer to only create path for only 15 seconds
-  now lets use b spline algorithm for waypoints

- Tracked Color from Centroid:
    - Implemented logic to detect a specific color (e.g., yellow) using HSV masks.
    - Used contours to calculate the centroid of the detected color, which represented the object's position.
- Created Path of the Color:
    - Appended the centroid positions to a list to create a path reflecting the movement of the tracked object.
    - Visualized the path on the image as a green line connecting the centroids.

- Attempted Path Smoothing with Moving Average Filter: 
    - Tried smoothing the path by averaging the nearest neighbors of each point using a moving average filter.
    
    - Found that this caused unwanted merging of points and altered the unique shape of the path.

- Resolved Path Issues by Merging Contours:
    - Instead of smoothing the path, merged nearby contours to stabilize the centroid calculation.
    - Pushkar suggested that path smoothing can be handled later during waypoint generation, simplifying the color tracking node.

- Set a Timer for Path Creation:
    - Limited path recording to 15 seconds using a timer in the color tracking node.
    Ensured that the node stops appending points to the path after the time limit.

- Started Using B-Spline Algorithm for Waypoints:
    - Decided to smooth the path and generate waypoints using a B-spline algorithm in the waypoint generation node.




### Friday Jan 24 
- Used b spline snf got pretty good waypoint generation 
- filtered out redundant points --> if the marker was in the same spot too many points were unnecesarrily gernerated 
- path stops when yellow moves out of frame 

### Saturday Jan 25
- Tried to get waypoints to turtlebot 
- Used cross compilation to try to get waypoints to turtlebot 
- Used the aarch64 Docker container to cross-compile nuturtlebot_msgs and other packages for the TurtleBot.
- Transferred the cross-compiled files (aarch64_install) to the TurtleBot and sourced the setup script.
- Tried running the waypoint_follower node on the TurtleBot but encountered errors related to the missing Python bindings for nuturtlebot_msgs.


## Monday Jan 27 
- continued debugging the message import issue, making little progress early in the day.
- Realized the issue might stem from a mismatch between the Python-based waypoint follower and the C++-based package dependencies.
- Spent the entire day troubleshooting with Alan on Discord, diagnosing the root cause of the problem. 

## Tuesday Jan 28 
- tried to debug msg issue again bad day 
- realized issue was  maybe something about my waypoint followe being python but package being c ++ 
- spent whole day debugging with alan on discord 

ERROR MESSAGE: 

msr@shredder:~/install$ ls -l ~/install/nuturtlebot_msgs/lib
total 196
-rw-r--r-- 1 msr msr 29480 Jan 29 23:16 libnuturtlebot_msgs__rosidl_generator_c.so
-rw-r--r-- 1 msr msr 22560 Jan 29 23:16 libnuturtlebot_msgs__rosidl_generator_py.so
-rw-r--r-- 1 msr msr 14784 Jan 29 23:16 libnuturtlebot_msgs__rosidl_typesupport_c.so
-rw-r--r-- 1 msr msr 15616 Jan 29 23:16 libnuturtlebot_msgs__rosidl_typesupport_cpp.so
-rw-r--r-- 1 msr msr 25032 Jan 29 23:16 libnuturtlebot_msgs__rosidl_typesupport_fastrtps_c.so
-rw-r--r-- 1 msr msr 27288 Jan 29 23:16 libnuturtlebot_msgs__rosidl_typesupport_fastrtps_cpp.so
-rw-r--r-- 1 msr msr 16784 Jan 29 23:16 libnuturtlebot_msgs__rosidl_typesupport_introspection_c.so
-rw-r--r-- 1 msr msr 25320 Jan 29 23:16 libnuturtlebot_msgs__rosidl_typesupport_introspection_cpp.so
drwxr-xr-x 3 msr msr  4096 Jan 29 23:16 python3.10
msr@shredder:~/install$ source ~/install/setup.bash
source /opt/ros/iron/setup.bash
msr@shredder:~/install$ ros2 run me495_yolo waypoint_follower
Traceback (most recent call last):
  File "/opt/ros/iron/lib/python3.10/site-packages/rosidl_generator_py/import_type_support_impl.py", line 46, in import_type_support
    return importlib.import_module(module_name, package=pkg_name)
  File "/usr/lib/python3.10/importlib/__init__.py", line 126, in import_module
    return _bootstrap._gcd_import(name[level:], package, level)
  File "<frozen importlib._bootstrap>", line 1050, in _gcd_import
  File "<frozen importlib._bootstrap>", line 1027, in _find_and_load
  File "<frozen importlib._bootstrap>", line 1004, in _find_and_load_unlocked
ModuleNotFoundError: No module named 'nuturtlebot_msgs.nuturtlebot_msgs_s__rosidl_typesupport_c'

During handling of the above exception, another exception occurred:

Traceback (most recent call last):
  File "/home/msr/install/me495_yolo/lib/me495_yolo/waypoint_follower", line 33, in <module>
    sys.exit(load_entry_point('me495-yolo==0.0.0', 'console_scripts', 'waypoint_follower')())
  File "/home/msr/install/me495_yolo/lib/python3.10/site-packages/me495_yolo/waypoint_follower.py", line 85, in main
    node = WaypointFollower()
  File "/home/msr/install/me495_yolo/lib/python3.10/site-packages/me495_yolo/waypoint_follower.py", line 18, in __init__
    self.wheel_cmd_publisher = self.create_publisher(WheelCommands, '/wheel_cmd', 10)
  File "/opt/ros/iron/lib/python3.10/site-packages/rclpy/node.py", line 1486, in create_publisher
    check_is_valid_msg_type(msg_type)
  File "/opt/ros/iron/lib/python3.10/site-packages/rclpy/type_support.py", line 35, in check_is_valid_msg_type
    check_for_type_support(msg_type)
  File "/opt/ros/iron/lib/python3.10/site-packages/rclpy/type_support.py", line 29, in check_for_type_support
    msg_or_srv_type.__class__.__import_type_support__()
  File "/home/msr/install/nuturtlebot_msgs/lib/python3.10/site-packages/nuturtlebot_msgs/msg/_wheel_commands.py", line 36, in __import_type_support__
    module = import_type_support('nuturtlebot_msgs')
  File "/opt/ros/iron/lib/python3.10/site-packages/rosidl_generator_py/import_type_support_impl.py", line 48, in import_type_support
    raise UnsupportedTypeSupport(pkg_name)
rosidl_generator_py.import_type_support_impl.UnsupportedTypeSupport: Could not import 'rosidl_typesupport_c' for package 'nuturtlebot_msgs'
[ros2run]: Process exited with failure 1
msr@shredder:~/install$ ros2 pkg list | grep nuturtlebot_msgs
nuturtlebot_msgs

Library exists: libnuturtlebot_msgs__rosidl_typesupport_c.so is present.
- ROS package recognized: ros2 pkg list | grep nuturtlebot_msgs finds it.
- Python import works: python3 -c "import nuturtlebot_msgs" runs without error.
- Correct architecture: file ... confirms ARM aarch64.

So why does ROS2 still say "Could not import 'rosidl_typesupport_c' for package 'nuturtlebot_msgs'"?

## Wednesday Jan 29 
- better day 
- Made significant progress by setting the ROS domain ID to match between the TurtleBot and the local machine, allowing the waypoint_follower to run successfully.
- Decided to shift all processing to local execution instead of running it on the TurtleBot directly.
- Used wheel_cmd commands to approximate the TurtleBot’s movement and visualize the general shape of its trajectory.

## Thursday Jan 30 
- Key realization: The TurtleBot needs a colored marker for accurate frame tracking and navigation.
- day off

## Friday Jan 31
- Successfully set up TF transformations from the camera to the TurtleBot and from the camera to the dancer.
- Began finalizing waypoint generation in the TurtleBot's frame.
- Explored scaling transformations using the Z-depth value to account for the camera’s field of view.
- Investigated how to use Z-depth for reconstruction and determine appropriate displacement scaling.
- Considered using a reference-based scaling approach, aligning the center of the trajectory with the TurtleBot’s movement.

NOTES FROM CHECK IN
- transformations set up for camera to turtlebot and camera to dancer
- nowp finishing gettign waypoints in turtle frame 
- add scaling to figure out --> use z for scaling for field of view 
- use z to reconstruct 
- pick displacement if i want it to scale
- scale with referenfe --> fidn center of trajectory -> scaling them move turtlebot so one point on traj corr with turtlebot 



## Saturday Feb 1 
- Attempted to ensure all frames were properly detected and tracked, but ... 
-Issue: Waypoints were not appearing when echoing /waypoints.
-Suspected cause: TF transformations were not consistently available, leading to failures when transforming waypoints into the TurtleBot’s frame.
## Sunday Feb 2 
- Confirmed that /path_points is correctly publishing X, Y, Z coordinates in the camera frame.
- Verified that TF transformations exist between camera_color_optical_frame and turtlebot_base, but still encountered errors.
Persistent issue:

- When running ros2 topic echo /waypoints, the topic remains empty.
    Running ros2 run me495_yolo waypoint outputs transformed waypoints, but they are not being published.
    Encountered "lookupTransform argument target_frame does not exist" errors, indicating that turtlebot_base might not always be available when the transform is requested.

Possible cause: The dancer’s frame only exists when detected, so if the RealSense camera momentarily loses sight of the pink object, TF transformations fail.

- Next steps: Ensure continuous TF broadcasting for the dancer’s frame, even if temporarily lost. Add better error handling for missing TF lookups instead of skipping waypoints. Confirm that publish_b_spline_path() is actually executing and sending waypoints to /waypoints.

## Monday Feb 3
- Still working on getting waypoints. They dont publish or echo but are there 

## Tuesday Feb 4 
- Debugged waypoint transformation pipeline.
- Ensured waypoints appeared correctly in RViz.
- Adjusted TF transformations to properly reference the TurtleBot’s frame.
- Successfully got /waypoints to publish in the correct frame.


## Wednesday Feb 5
- Worked on making the TurtleBot follow waypoints.
- Verified /waypoints structure and ensured the follower node was receiving them.
- Incorporated TurtleBot’s detected position (blue object in YOLO) as a navigation reference.
- Wrote initial waypoint_follower.py to process and execute waypoints.
Added a new topic /turtlebot_position to update the TurtleBot’s location from YOLO.
-Debugged issues with receiving and processing position updates.

## Thursday Feb 6 
- added missing topic /turtlebot_position
- Ensured YOLO was correctly publishing the TurtleBot’s coordinates
- Verified waypoint_follower.py was subscribing to /turtlebot_position correctly.
Fixed crashes related to missing callback functions.
- time to send waypoitns to turtlebot!!! lets gooooooo !!!!!!!! 


## Week of Feb 10th:

- after two more days of tryign to figure out the rodidl error in matts package I decided to abandon hi spackage and just use turtlebot startup to launch after testing every turtlebot

- sample set up with ladder and got turtlebots moving. confirmed setup for video

- during sample set up the turtlebots were jsut moving in circles 

- to figure out why they were moving in circles I checked all transformations and checked turtlebot position tracking accuracy 

- realized that incorrect orientation is the problem --> used itnernal odom but not accurate enough --> thinking about optitrack, thinking about april tags 

- re pauls recommendation going to add a second color to the front of the turtlebot for orientation to determine yaw angle 

- by using two markers i can comoute the orientation fo the turtlebot relative to the camera --> fidn vector from blue marker to yellow marker (yellow marker will be on the front of turtlebot) --> calculate orientation usign atan2 to determine headign directin --> convert heading direction to turtlrebot frame to use for movement control 

## Week of Feb 17th:

- Calculated TurtleBot orientation using an orange marker at the front. Tracking shows some success, and the TurtleBot can orient itself at times, but it seems to fail when color detection is inconsistent.
- Experimented with using AprilTags instead of color detection. The goal was to track the dancer’s position. I can now visualize AprilTags and get position data in RViz, but fully integrating them into my existing code is more time-intensive than expected, so I’ve paused this for now.
- Ran a sanity check on basic square movement using direct cmd_vel commands.
- Working on basic square movement using waypoints. Once this is functional, I’ll return to using the pink object for waypoint tracking.


### Week of Feb 24 
- april tag based navigation!!
- Subscribed to AprilTag detections for TurtleBot center, front, and waypoints.
- Used TF transforms to convert waypoints from camera frame → TurtleBot frame.
- Implemented co-linearity checks to ensure the robot correctly aligns before moving forward.
- Added a distance check to ensure the TurtleBot is facing towards the waypoint rather than away.


## Week of March 3
- wrote hsv tuning code --> this is works realls well
- the goal this week is to re implement color trackign now that i have colinearity adn frames working
- got solid video of turltbot moving to path based on color

## Week  of March 10
-edited demo videos
- created two python files for light trails
- one python file shows the color overlay with the video in the background
- the next python file filters on only shows the ligth trail of the hsv values i enter (glow in the dark style video)
- began implementing feedback control 



## Week  of March 17
- finished portfolio post
- cleaned up code and pushed to git
- wrote extensive read me so i can revisit this progress and add to it
- tryign to get one more good video with better lighting just for better image quality for portfolio 

