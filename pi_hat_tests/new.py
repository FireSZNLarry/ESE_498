import math
import os
import time
import pygame
import busio
from math import cos, sin, pi, floor
from board import SCL, SDA
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
from adafruit_rplidar import RPLidar

i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 100
channel_num = 14
servo7 = servo.Servo(pca.channels[channel_num])

# Set up pygame for LIDAR visualization
os.putenv('SDL_FBDEV', '/dev/fb1')
pygame.init()
lcd = pygame.display.set_mode((320, 240))
pygame.mouse.set_visible(False)
lcd.fill((0, 0, 0))
pygame.display.update()

# LIDAR setup
PORT_NAME = '/dev/ttyUSB0'
lidar = RPLidar(None, PORT_NAME, timeout=10000)

# Motor and Servo setup
i2c_bus = busio.I2C(SCL, SDA)
pca = PCA9685(i2c_bus)
pca.frequency = 100
motor_channel = 15
steering_channel = 14

servo_steering = servo.Servo(pca.channels[steering_channel])
motor = pca.channels[motor_channel]


# Helper function to update steering angle
def update_steering_angle(angle):
    servo_steering.angle = angle

# Helper function to scale LIDAR distance data
def scale_lidar_distance(distance, max_distance=100000):
    return min(distance, max_distance) / max_distance
    print("hi")



while True:
    scan_data = [0]*360
    for scan in lidar.iter_scans():
        for (_, angle, distance) in scan:
            angle = int(angle)
            if 120 <= angle < 160:
                scan_data[angle] = distance
                if distance < 500:
                    print(distance)
                    update_steering_angle(70)
                    time.sleep(0.1)
                elif distance < 1000:
                    print(distance)
                    update_steering_angle(130)
                    time.sleep(0.1)
                else:
                    print(distance)
                    update_steering_angle(90)
                    time.sleep(0.1)

lidar.stop()
lidar.disconnect()



