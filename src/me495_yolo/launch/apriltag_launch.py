# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
#     # Path to the configuration file
#     config_file = os.path.join(
#         get_package_share_directory('apriltag_ros'),
#         'config',
#         'tags.yaml'
#     )

#     # AprilTag detector node
#     apriltag_node = Node(
#         package='apriltag_ros',
#         executable='apriltag_node',
#         name='apriltag_node',
#         output='screen',
#         parameters=[config_file],
#         remappings=[
#             ('/image_rect', '/camera/camera/color/image_raw'),
#             ('/camera_info', '/camera/camera/color/camera_info')
#         ],
#         parameters=[
#             {"image_transport": "raw"},  # Force raw image transport
#             {"use_sim_time": False}       # Ensure real-time synchronization
#         ]

#     )

#     # Your custom AprilTag detector node
#     apriltag_detector_node = Node(
#         package='me495_yolo',   # Your package name
#         executable='apriltag_detector',  # Your script name
#         name='apriltag_detector',
#         output='screen'
#     )

#     return LaunchDescription([apriltag_node])

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Path to the configuration file
    config_file = os.path.join(
        get_package_share_directory('me495_yolo'),
        'config',
        'tags.yaml'
    )

    # AprilTag detector node (from `apriltag_ros`)
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        output='screen',
        parameters=[
            config_file,
            {"image_transport": "raw"},  # Force raw image transport
            {"use_sim_time": False}       # Ensure real-time synchronization
        ],
        remappings=[
            ('/image_rect', '/camera/camera/color/image_raw'),
            ('/camera_info', '/camera/camera/color/camera_info')
        ]
    )

    # Your custom AprilTag detector node
    apriltag_detector_node = Node(
        package='me495_yolo',   # Your package name
        executable='apriltag_detector',  # Your script name
        name='apriltag_detector',
        output='screen'
    )

    return LaunchDescription([
        apriltag_node, 
        apriltag_detector_node  # ✅ Now included in the return statement
    ])
