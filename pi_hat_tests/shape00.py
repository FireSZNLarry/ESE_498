import cv2
import numpy as np
from matplotlib import pyplot as plt

# Capture video from the camera (video0)
cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)

# Set dimensions
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1440)

# Take a frame
ret, frame = cap.read()

# Flip the frame to display it right side up
flipped_frame = cv2.flip(frame, 0)

# Write the frame to a file
cv2.imwrite('image1.jpg', flipped_frame)

# Release the camera
cap.release()

# Read the saved image
img = cv2.imread('image1.jpg')

# Convert the image to grayscale
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Apply thresholding to create a binary image
_, threshold = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

# Find contours in the binary image
contours, _ = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

# Initialize a counter
i = 0

# List for storing shape names
for contour in contours:
    # Ignore the first contour (whole image)
    if i == 0:
        i = 1
        continue
    if i == 2:
        end

    # Approximate the shape using cv2.approxPolyDP()
    #epsilon = 1 * cv2.arcLength(contour, True)
    epsilon = 0.01 * cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, epsilon, True)

    # Draw contours
    cnt = contours[4]
    cv.drawContours(img, [cnt], 0, (0,255,0), 3)
    #cv2.drawContours(img, [contour], 0, (0, 0, 255), 5)

    # Calculate the center point of the shape
    M = cv2.moments(contour)
    if M['m00'] != 0.0:
        x = int(M['m10'] / M['m00'])
        y = int(M['m01'] / M['m00'])

    # Put the shape name at the center of each shape
    if len(approx) == 3:
        cv2.putText(img, 'Triangle', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    elif len(approx) == 4:
        cv2.putText(img, 'Quadrilateral', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    elif len(approx) == 5:
        cv2.putText(img, 'Pentagon', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    elif len(approx) == 6:
        cv2.putText(img, 'circle', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2) 

# Display the result as a JPEG image
cv2.imwrite('processed_image.jpg', img)

# Release resources
cv2.destroyAllWindows()

# Show the processed image
plt.imshow(img)
plt.axis('off')  # Hide axes
plt.show()
