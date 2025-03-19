# import cv2
# import numpy as np

# # Load the video
# video_path = '/home/catherine-maglione/yolo_backup/videos/pink_and_green.mp4'  # Corrected path
# cap = cv2.VideoCapture(video_path)

# # Check if the video loads correctly
# if not cap.isOpened():
#     print("⚠️ ERROR: Could not read video! Check the file path or format.")
#     exit()

# # Get video properties
# frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
# frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
# frame_rate = int(cap.get(cv2.CAP_PROP_FPS))

# print(f"✅ Video loaded successfully! Frame size: {frame_width}x{frame_height}, FPS: {frame_rate}")

# # Create an output video writer
# output_path = '/home/catherine-maglione/pink_and_green_dark.mp4'
# out = cv2.VideoWriter(output_path, 
#                       cv2.VideoWriter_fourcc(*'mp4v'), 
#                       frame_rate, 
#                       (frame_width, frame_height))

# # Initialize an accumulation canvas with a black background
# light_painting = np.zeros((frame_height, frame_width, 3), dtype=np.uint8)

# frame_count = 0  # Track frame numbers
# while cap.isOpened():
#     ret, frame = cap.read()
#     if not ret:
#         print("End of video reached.")
#         break

#     frame_count += 1
#     print(f"Processing frame {frame_count}...")

#     # Convert to HSV for color detection
#     hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
#     # Define HSV ranges to track pink/magenta colors
#     color_ranges = [
#         (np.array([130, 50, 230]), np.array([165, 255, 255]))  # Pink/Magenta
#         #(np.array([45, 40, 140]), np.array([90, 255, 255]))  # Green

#     ]
    
#     combined_mask = np.zeros((frame_height, frame_width), dtype=np.uint8)
#     for lower_bound, upper_bound in color_ranges:
#         mask = cv2.inRange(hsv, lower_bound, upper_bound)
#         combined_mask = cv2.bitwise_or(combined_mask, mask)
    
#     # Debug: Check if anything is detected
#     if np.sum(combined_mask) == 0:
#         print(f"⚠️ Frame {frame_count}: No colors detected in the defined HSV range!")
    
#     # Extract the glowing parts
#     glowing_parts = cv2.bitwise_and(frame, frame, mask=combined_mask)
    
#     # Accumulate the detected marker trails on a black background
#     light_painting = cv2.add(light_painting, glowing_parts)
    
#     # Save only the light path (black background with accumulated light trails)
#     out.write(light_painting)
#     print("✅ Writing frame to output video...")
    
#     # Show the accumulated light path for debugging
#     cv2.imshow('Light Painting Effect', light_painting)
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# # Cleanup
# cap.release()
# out.release()
# cv2.destroyAllWindows()

# print(f"Long exposure light painting video saved as: {output_path}")

import cv2
import numpy as np

# Load the video
video_path = '/home/catherine-maglione/yolo_backup/videos/pink_and_green.mp4'  # Corrected path
cap = cv2.VideoCapture(video_path)

# Check if the video loads correctly
if not cap.isOpened():
    print("⚠️ ERROR: Could not read video! Check the file path or format.")
    exit()

# Get video properties
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
frame_rate = int(cap.get(cv2.CAP_PROP_FPS))

print(f"✅ Video loaded successfully! Frame size: {frame_width}x{frame_height}, FPS: {frame_rate}")

# Create an output video writer
output_path = '/home/catherine-maglione/pink_and_green_dark.mp4'
out = cv2.VideoWriter(output_path, 
                      cv2.VideoWriter_fourcc(*'mp4v'), 
                      frame_rate, 
                      (frame_width, frame_height))

# Initialize an accumulation canvas with a black background
light_painting = np.zeros((frame_height, frame_width, 3), dtype=np.uint8)

# Minimum area threshold to filter out small detections
min_contour_area = 100  # Adjust this value as needed

frame_count = 0  # Track frame numbers
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print("End of video reached.")
        break

    frame_count += 1
    print(f"Processing frame {frame_count}...")

    # Convert to HSV for color detection
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Define HSV ranges to track both pink/magenta and green colors
    color_ranges = [
        (np.array([130, 50, 230]), np.array([165, 255, 255])),  # Pink/Magenta
        (np.array([45, 40, 140]), np.array([90, 255, 255]))  # Green
    ]
    
    combined_mask = np.zeros((frame_height, frame_width), dtype=np.uint8)
    for lower_bound, upper_bound in color_ranges:
        mask = cv2.inRange(hsv, lower_bound, upper_bound)
        combined_mask = cv2.bitwise_or(combined_mask, mask)
    
    # Find contours to filter out small detections
    contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    filtered_mask = np.zeros_like(combined_mask)
    for contour in contours:
        if cv2.contourArea(contour) > min_contour_area:
            cv2.drawContours(filtered_mask, [contour], -1, 255, thickness=cv2.FILLED)
    
    # Debug: Check if anything is detected
    if np.sum(filtered_mask) == 0:
        print(f"⚠️ Frame {frame_count}: No significant colors detected in the defined HSV range!")
    
    # Extract the glowing parts
    glowing_parts = cv2.bitwise_and(frame, frame, mask=filtered_mask)
    
    # Apply Gaussian blur to make color trails smoother and less pixelated
    glowing_parts = cv2.GaussianBlur(glowing_parts, (5, 5), 0)
    
    # Enhance saturation to make colors more vivid
    hsv_glowing = cv2.cvtColor(glowing_parts, cv2.COLOR_BGR2HSV)
    hsv_glowing[:, :, 1] = cv2.add(hsv_glowing[:, :, 1], 50)  # Increase saturation
    glowing_parts = cv2.cvtColor(hsv_glowing, cv2.COLOR_HSV2BGR)
    
    # Accumulate the detected marker trails on a black background
    light_painting = cv2.add(light_painting, glowing_parts)
    
    # Save only the light path (black background with accumulated light trails)
    out.write(light_painting)
    print("✅ Writing frame to output video...")
    
    # Show the accumulated light path for debugging
    cv2.imshow('Light Painting Effect', light_painting)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
cap.release()
out.release()
cv2.destroyAllWindows()

print(f"Long exposure light painting video saved as: {output_path}")