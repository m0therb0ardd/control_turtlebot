import cv2
import numpy as np

# Load the video
video_path = '/home/catherine-maglione/yolo_backup/videos/push_turtle_path_fast.mp4'  # Corrected path
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
output_path = '/home/catherine-maglione/push_turtle_dark.mp4'
out = cv2.VideoWriter(output_path, 
                      cv2.VideoWriter_fourcc(*'mp4v'), 
                      frame_rate, 
                      (frame_width, frame_height))

# Initialize an accumulation canvas with a black background
light_painting = np.zeros((frame_height, frame_width, 3), dtype=np.uint8)

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
    
    # Define HSV ranges to track pink/magenta colors
    color_ranges = [
        #(np.array([140, 40, 100]), np.array([170, 255, 255]))  # Pink/Magenta
        (np.array([45, 40, 140]), np.array([90, 255, 255]))  # Green

    ]
    
    combined_mask = np.zeros((frame_height, frame_width), dtype=np.uint8)
    for lower_bound, upper_bound in color_ranges:
        mask = cv2.inRange(hsv, lower_bound, upper_bound)
        combined_mask = cv2.bitwise_or(combined_mask, mask)
    
    # Debug: Check if anything is detected
    if np.sum(combined_mask) == 0:
        print(f"⚠️ Frame {frame_count}: No colors detected in the defined HSV range!")
    
    # Extract the glowing parts
    glowing_parts = cv2.bitwise_and(frame, frame, mask=combined_mask)
    
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