import pyrealsense2 as rs
import numpy as np
import cv2
import os

# Set up directories
save_dir = "../dataset/images/"
os.makedirs(save_dir, exist_ok=True)

# Initialize camera pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

pipeline.start(config)

color_name = input("Enter the color label (e.g., red, blue, green): ")
image_count = 0
max_images = 70  # Change as needed

try:
    while image_count < max_images:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())

        filename = os.path.join(save_dir, f"{color_name}_{image_count}.jpg")
        cv2.imwrite(filename, color_image)

        cv2.imshow("Captured Image", color_image)
        cv2.waitKey(1)

        print(f"Captured {image_count + 1}/{max_images} images for {color_name}")
        image_count += 1

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
