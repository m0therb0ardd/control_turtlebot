# import cv2
# import numpy as np

# # Load the video
# video_path = 'videos/turtle2.mp4'  # Change this 
# cap = cv2.VideoCapture(video_path)

# # Get video properties
# frame_width = int(cap.get(3))
# frame_height = int(cap.get(4))
# frame_rate = int(cap.get(cv2.CAP_PROP_FPS))

# # Create an output video writer
# out = cv2.VideoWriter('light_painting_output.mp4', 
#                        cv2.VideoWriter_fourcc(*'mp4v'), 
#                        frame_rate, 
#                        (frame_width, frame_height))

# # Initialize an accumulation canvas
# light_painting = np.zeros((frame_height, frame_width, 3), dtype=np.uint8)

# while cap.isOpened():
#     ret, frame = cap.read()
#     if not ret:
#         break  # Break if video ends
    
#     # Convert to HSV for color detection
#     hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
#     # Define color range (adjust based on your glowing marker color)
#     #tracking green right now
#     lower_bound = np.array([30, 40, 140])  # Adjust this for your marker color
#     upper_bound = np.array([90, 255, 255])
    
#     # Create a mask
#     mask = cv2.inRange(hsv, lower_bound, upper_bound)
    
#     # Extract the glowing marker
#     glowing_parts = cv2.bitwise_and(frame, frame, mask=mask)
    
#     # Add the detected marker to the accumulation canvas
#     light_painting = cv2.addWeighted(light_painting, 0.9, glowing_parts, 0.1, 0)
    
#     # Save frame with overlay
#     output_frame = cv2.addWeighted(frame, 0.5, light_painting, 0.5, 0)
#     out.write(output_frame)
    
#     # Display (optional, can slow down processing)
#     cv2.imshow('Light Painting Effect', output_frame)
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# # Cleanup
# cap.release()
# out.release()
# cv2.destroyAllWindows()

# print("Long exposure light painting video saved as 'light_painting_output.mp4'")



import cv2
import numpy as np

# Load the video
video_path = 'videos/turtle2.mp4'  # Adjust this if needed
cap = cv2.VideoCapture(video_path)

# Get video properties
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
frame_rate = int(cap.get(cv2.CAP_PROP_FPS))

# Create an output video writer
out = cv2.VideoWriter('light_painting_output.mp4', 
                       cv2.VideoWriter_fourcc(*'mp4v'), 
                       frame_rate, 
                       (frame_width, frame_height))

# Initialize an accumulation canvas
light_painting = np.zeros((frame_height, frame_width, 3), dtype=np.uint8)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break  # Break if video ends
    
    # Convert to HSV for color detection
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Define multiple color ranges to track all glowing colors
    color_ranges = [
        #(np.array([0, 50, 50]), np.array([10, 255, 255])),   # Red (lower range)
        #(np.array([170, 50, 50]), np.array([180, 255, 255])), # Red (upper range)
         (np.array([30, 40, 140]), np.array([90, 255, 255])),  # Green
        # (np.array([100, 50, 50]), np.array([140, 255, 255]))  # Blue
    ]
    
    combined_mask = np.zeros((frame_height, frame_width), dtype=np.uint8)
    for lower_bound, upper_bound in color_ranges:
        mask = cv2.inRange(hsv, lower_bound, upper_bound)
        combined_mask = cv2.bitwise_or(combined_mask, mask)
    
    # Extract the glowing parts
    glowing_parts = cv2.bitwise_and(frame, frame, mask=combined_mask)
    
    # Accumulate the detected marker trails
    light_painting = cv2.add(light_painting, glowing_parts)
    
    # Save frame with overlay
    output_frame = cv2.addWeighted(frame, 0.5, light_painting, 0.5, 0)
    out.write(output_frame)
    
    # Display (optional, can slow down processing)
    cv2.imshow('Light Painting Effect', output_frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
cap.release()
out.release()
cv2.destroyAllWindows()

print("Long exposure light painting video saved as 'light_painting_output.mp4'")

