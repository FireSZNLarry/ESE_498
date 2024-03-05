import cv2
from matplotlib import pyplot as plt

# Load the cascade classifier for shape detection
stop_data = cv2.CascadeClassifier('stop_data.xml')

# Read the image
img = cv2.imread("flipped_image.jpg")
img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Detect shapes
found = stop_data.detectMultiScale(img_gray, minSize=(20, 20))

# Check if any shapes were found
amount_found = len(found)

if amount_found != 0:
    # Draw rectangles around detected shapes
    for (x, y, width, height) in found:
        cv2.rectangle(img, (x, y), (x + width, y + height), (0, 255, 0), 5)

    # Display the result
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    plt.imshow(img_rgb)
    plt.show()
else:
    print("No shapes detected in the image.")

# Release resources
cv2.destroyAllWindows()
