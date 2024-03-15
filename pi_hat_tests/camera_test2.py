import cv2

# Open camera
cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)

# Check if camera opened successfully
if not cap.isOpened():
    print("Error opening camera. Make sure it is connected and accessible.")
    exit()

# Set dimensions
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1440)

# Take a frame
ret, frame = cap.read()

# Check if frame is captured successfully
if not ret:
    print("Error capturing frame.")
    cap.release()
    exit()

# Flip the frame (takes pictures right side up)
flipped_frame = cv2.flip(frame, 0)

# Write the frame to a file
cv2.imwrite('flipped_image.jpg', flipped_frame)

# Release the camera
cap.release()

print("Flipped image saved as flipped_image.jpg")
