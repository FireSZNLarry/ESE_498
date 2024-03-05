# Code from https://www.geeksforgeeks.org/how-to-detect-shapes-in-images-in-python-using-opencv/

import cv2 

cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)

# set dimensions
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1440)

# take frame
ret, frame = cap.read()
#flip frame takes pictures right side up
flipped_frame = cv2.flip(frame,0)

# write frame to file
cv2.imwrite('image1.jpg', flipped_frame)
# release camera
cap.release()

#reading the image  
image = cv2.imread("image1.jpg") 
edged = cv2.Canny(image, 10, 250) 
cv2.imshow("Edges", edged) 
cv2.waitKey(0) 
 
#applying closing function  
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7)) 
closed = cv2.morphologyEx(edged, cv2.MORPH_CLOSE, kernel) 
cv2.imshow("Closed", closed) 
cv2.waitKey(0) 
 
#finding_contours  
(cnts, _) = cv2.findContours(closed.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) 
 
for c in cnts: 
	peri = cv2.arcLength(c, True) 
	approx = cv2.approxPolyDP(c, 0.02 * peri, True) 
	cv2.drawContours(image, [approx], -1, (0, 255, 0), 2) 
cv2.imshow("Output", image) 
cv2.waitKey(0) 
