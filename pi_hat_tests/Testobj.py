import numpy as np
import cv2


# read the image
image = cv2.imread("examples/1.jpg")
# convert from BGR to HSV color space
hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# lower and upper limits for the white color
lower_limit = np.array([75, 0, 99])
upper_limit = np.array([179, 62, 255])

# create a mask for the specified color range
mask = cv2.inRange(hsv_image, lower_limit, upper_limit)
# get the bounding box from the mask image
bbox = cv2.boundingRect(mask)
