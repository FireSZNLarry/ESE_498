import cv2
import numpy as np
import time
from time import sleep
import math
from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
import os
import sys
from math import cos, sin, pi, floor
import pygame
from adafruit_rplidar import RPLidar

# Initialize systems for LIDAR and servo control using PCA9685
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 100
servo_steering = servo.Servo(pca.channels[14])
motor_channel = 15

def motor_speed(pca, percent, reverse=False):

    if reverse:
        speed = ((-abs(percent)) * 3277) + 65535 * 0.15  # Reverse speed calculation
    else:
        speed = (-abs(percent) * 3277) + 65535 * 0.15     # Forward speed calculation
    pca.channels[motor_channel].duty_cycle = math.floor(speed)

def update_steering_angle(angle):

    servo_steering.angle = angle

def scale_lidar_distance(distance, max_distance=3000):

    return min(distance, max_distance) / max_distance
    

def main():
    try:
        # Camera setup
        cap = cv2.VideoCapture(0)
        cap.set(3, 320)
        cap.set(4, 240)
        
        # Set up environment vars for SDL to use the Pi's TFT screen
        os.putenv('SDL_FBDEV', '/dev/fb1')
        pygame.init()
        
        # LIDAR setup
        PORT_NAME = '/dev/ttyUSB0'
        lidar = RPLidar(None, PORT_NAME, timeout=3)
        
        while True:
            ret, frame = cap.read()
            frame = cv2.blur(frame, (3,3))
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            thresh = cv2.inRange(hsv, np.array((75, 80, 80)), np.array((90, 255, 255))) # Generic threshold for visual obstacle
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Choose the biggest contour as the main object
            if contours:
                c = max(contours, key=cv2.contourArea)
                x, y, w, h = cv2.boundingRect(c)
                cx = x + w // 2
                object_detected = True
            else:
                object_detected = False
            
            scan_data = [0]*360
            for scan in lidar.iter_scans():
                update_steering_angle(90)  # center the steering initially
                for (_, angle, distance) in scan:
                    if object_detected and 1000 < distance < 2000:
                        # Reverse if close to object
                        motor_speed(pca, 0.15, reverse=True)
                        sleep(0.5)  # continue reversing for a bit
                        motor_speed(pca, 0)  # stop to assess situation
                        if cx > 160:  # Object is on the right
                            update_steering_angle(60)  # steer left
                        else:
                            update_steering_angle(120)  # steer right
                    else:
                        # Normal forward movement
                        motor_speed(pca, 0.15, reverse=False)
                
    except KeyboardInterrupt:
        print("Program stopped by user.")
    finally:
        cv2.destroyAllWindows()
        cap.release()
        lidar.stop()
        lidar.disconnect()
        pygame.quit()

if __name__ == "__main__":
    main()
