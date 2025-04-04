from setuptools import find_packages, setup

package_name = 'me495_yolo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml','launch/apriltag_launch.py','config/tags.yaml']),
        # ('launch/apriltag_launch.py'),
        # ('config/tags.yaml')
    ],
    # install_requires=[
    #     'setuptools',
    #     'numpy',
    #     'scipy',
    #     'opencv-python',  # Ensure OpenCV is listed here
    # ],
    zip_safe=True,
    maintainer='elwin',
    maintainer_email='elwin@northwestern.edu',
    description='ROS2 package for YOLO-based object detection and waypoint navigation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo = me495_yolo.yolo:main',
            'waypoint = me495_yolo.waypoint:main',
            'waypoint_follower = me495_yolo.waypoint_follower:main',
            'square_motion = me495_yolo.square_motion:main',
            'rotate_to_90= me495_yolo.rotate_to_90:main',
            'apriltag_detector = me495_yolo.apriltag_detector:main',
            'square_mover = me495_yolo.square_mover:main',
            'good_square = me495_yolo.good_square:main',
            'waypoint_april = me495_yolo.waypoint_april:main',
            'hsv_tuner = me495_yolo.hsv_tuner:main',
            'color_tracking = me495_yolo.color_tracking:main',
            
        ],
    },
)
