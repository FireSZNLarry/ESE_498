import os
from math import cos, sin, pi, floor
import pygame
from adafruit_rplidar import RPLidar

# Environment and display setup for using pygame
os.putenv('SDL_FBDEV', '/dev/fb1')
pygame.init()
lcd = pygame.display.set_mode((320, 240))
pygame.mouse.set_visible(False)
lcd.fill((0, 0, 0))
pygame.display.update()

# Initializing the LIDAR
PORT_NAME = '/dev/ttyUSB0'
lidar = RPLidar(None, PORT_NAME, timeout=3)

max_distance = 0  # Max distance to scale data to fit on the screen

def process_data(data):
    """
    Process and map the LIDAR data to generate a visual representation.

    Args:
        data: A list of distance measurements from the LIDAR.
    """
    global max_distance
    lcd.fill((0, 0, 0))  # Clear the screen for updated data points
    for angle in range(360):
        distance = data[angle]  # Get distance for the current angle
        if distance > 0:  # Filter out invalid measurements
            # Update max distance found, constrain to a maximum of 5000
            max_distance = max([min([5000, distance]), max_distance])
            radians = angle * pi / 180.0  # Convert angle to radians
            # Calculate x, y coordinates
            x = distance * cos(radians)
            y = distance * sin(radians)
            # Scale and translate points to fit them on the display
            point = (160 + int(x / max_distance * 119), 120 + int(y / max_distance * 119))
            lcd.set_at(point, pygame.Color(255, 255, 255))  # Draw the point
    pygame.display.update()  # Refresh the display to show updated data

scan_data = [0]*360  # Initialize scan data array

try:
    print(lidar.info())  # Display LIDAR information (might need adjustment to lidar.info() based on library version)
    for scan in lidar.iter_scans():
        for (_, angle, distance) in scan:
            # Update the scan data array with distances at corresponding angles
            scan_data[min([359, floor(angle)])] = distance
        process_data(scan_data)  # Process and display the updated scan data

except KeyboardInterrupt:
    print('Stopping.')
    lidar.stop()
    lidar.disconnect()
