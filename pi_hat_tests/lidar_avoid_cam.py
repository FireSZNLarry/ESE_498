import math
import os
import time
import pygame
import busio
from board import SCL, SDA
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
import cv2
from adafruit_rplidar import RPLidar
import numpy as np
from math import cos, sin, pi, floor




from time import sleep

#from picamera import PiCamera
import threading
import queue

i = 0
# Set up pygame for LIDAR visualization
os.putenv('SDL_FBDEV', '/dev/fb1')
pygame.init()
#lcd = pygame.display.set_mode((320, 240))
#pygame.mouse.set_visible(False)
#lcd.fill((0, 0, 0))
#pygame.display.update()

# LIDAR and Motor setup
PORT_NAME = '/dev/ttyUSB0'
lidar = RPLidar(None, PORT_NAME, timeout=3)
i2c_bus = busio.I2C(SCL, SDA)
pca = PCA9685(i2c_bus)
pca.frequency = 100
motor_channel = 15
steering_channel = 14
servo_steering = servo.Servo(pca.channels[steering_channel])
motor = pca.channels[motor_channel]

# Function to update motor speed
def update_motor_speed(speed):
    pwm_value = int((speed * 32767) + 32767)
    motor.duty_cycle = 0.5
    
# Function to update steering angle
def update_steering_angle(angle):
    servo_steering.angle = angle

# Helper function to scale LIDAR distance data
def scale_lidar_distance(distance, max_distance=4000):
    return min(distance, max_distance) / max_distance

def Servo_Motor_Initialization():
    i2c_bus = busio.I2C(SCL,SDA)
    pca = PCA9685(i2c_bus)
    pca.frequency = 100
    return pca

def Motor_Start(pca):
    x = input("Press and hold EZ button. Once the LED turns red, immediately relase the button. After the LED blink red once, press 'ENTER'on keyboard.")
    Motor_Speed(pca, 0.1)
    time.sleep(2)
    y = input("If the LED just blinked TWICE, then press the 'ENTER'on keyboard.")
    Motor_Speed(pca, -0.1)
    time.sleep(2)
    z = input("Now the LED should be in solid green, indicating the initialization is complete. Press 'ENTER' on keyboard to proceed")
   

def Motor_Speed(pca,percent):
    #converts a -1 to 1 value to 16-bit duty cycle
    speed = ((percent) * 3277) + 65535 * 0.15
    pca.channels[15].duty_cycle = math.floor(speed)
    #print(speed/65535)
       
#initialization
pca = Servo_Motor_Initialization()
Motor_Start(pca)

# Control parameters
safe_distance = 500  # Minimum distance from an obstacle in millimeters
backup_distance = 300  # Distance indicating too close, needing to back up
turn_angle = 110  # Angle to turn when avoiding an object 45
max_speed = 0.5

# Main loop with object avoidance
try:
    while i == 0:
        scan_data = [0]*360
        for scan in lidar.iter_scans():
            for (_, angle, distance) in scan:
                scan_data[min([359, math.floor(angle)])] = distance
            
            # Frontal distance
            front_distance = scan_data[0] if scan_data[0] != 0 else float('inf')

            # Check distances and react
            print(i)
            if front_distance < backup_distance:
                # Too close, back up and turn
                i = 1
                #servo7.angle = 70
                break
            elif front_distance < safe_distance:
                # Close, but not too close, just turn
                i = 2
                #servo7.angle = 135
                break
            else:
                # Safe distance, move forward
                i = 2
                #servo7.angle = 93
                break

            # Update pygame display with LIDAR data
            #lcd.fill((0, 0, 0))
            for angle in range(360):
                distance = scan_data[angle]
                if distance:
                    scaled_distance = scale_lidar_distance(distance)
                    radians = angle * math.pi / 180
                    x = scaled_distance * math.cos(radians) * 119
                    y = scaled_distance * math.sin(radians) * 119
                    point = (160 + int(x), 120 + int(y))
                    #lcd.set_at(point, pygame.Color(255, 255, 255))
            #pygame.display.update()
    while i == 1:
        print("i =1")
        Motor_Speed(pca, -0.15) 
        time.sleep(2)
        i = 0
    while i == 2:
        print("i =2")
        Motor_Speed(pca, 0.15) 
        time.sleep(2)
        i = 0
except KeyboardInterrupt:
    print('Stopping.')

finally:
    lidar.stop()
    lidar.disconnect()
    pygame.quit()
