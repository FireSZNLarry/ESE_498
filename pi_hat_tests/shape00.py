import cv2
import numpy as np

def detect_shapes(image_path):
    # Read the image
    image = cv2.imread(image_path, cv2.IMREAD_COLOR)

    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Apply edge detection (Canny)
    edges = cv2.Canny(gray, 50, 150)

    # Find contours in the edge-detected image
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Draw contours on the original image
    for contour in contours:
        epsilon = 0.04 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)

        if len(approx) == 3:
            shape_name = "Triangle"
        elif len(approx) == 4:
            shape_name = "Rectangle"
        elif len(approx) == 5:
            shape_name = "Pentagon"
        else:
            shape_name = "Circle"

        cv2.drawContours(image, [approx], 0, (0, 255, 0), 2)
        cv2.putText(image, shape_name, (approx[0][0][0], approx[0][0][1]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    # Display the result
    cv2.imshow("Detected Shapes", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    image_path = "path/to/your/image.jpg"  # Replace with the actual image path
    detect_shapes(image_path)
