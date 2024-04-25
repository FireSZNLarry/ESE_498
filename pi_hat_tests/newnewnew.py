# SPDX-FileCopyrightText: 2019 Dave Astels for Adafruit Industries
#
# SPDX-License-Identifier: MIT


"""
Consume LIDAR measurement file and create an image for display.

Adafruit invests time and resources providing this open source code.
Please support Adafruit and open source hardware by purchasing
products from Adafruit!

Written by Dave Astels for Adafruit Industries
Copyright (c) 2019 Adafruit Industries
Licensed under the MIT license.

All text above must be included in any redistribution.
"""

import os
from math import cos, sin, pi, floor
import pygame
from adafruit_rplidar import RPLidar
import math
import time
import busio
from board import SCL, SDA
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo


i2c_bus = busio.I2C(SCL, SDA)
pca = PCA9685(i2c_bus)
pca.frequency = 100
motor_channel = 15
steering_channel = 14

servo_steering = servo.Servo(pca.channels[steering_channel])
motor = pca.channels[motor_channel]

# Set up pygame and the display
os.putenv('SDL_FBDEV', '/dev/fb1')
pygame.init()
lcd = pygame.display.set_mode((320,240))
pygame.mouse.set_visible(False)
lcd.fill((0,0,0))
pygame.display.update()

# Setup the RPLidar
PORT_NAME = '/dev/ttyUSB0'
lidar = RPLidar(None, PORT_NAME,timeout=3)

# used to scale data to fit on the screen
max_distance = 0

def update_steering_angle(angle):
    servo_steering.angle = angle


#pylint: disable=redefined-outer-name,global-statement
def process_data(data):
    global max_distance
    lcd.fill((0,0,0))
    for angle in range(360):
        distance = data[angle]
        if distance > 0:                  # ignore initially ungathered data points
            if distance < 500:
                print(distance)
                update_steering_angle(70)
                time.sleep(0.1)
                max_distance = max([min([5000, distance]), max_distance])
                radians = angle * pi / 180.0
                x = distance * cos(radians)
                y = distance * sin(radians)
                point = (160 + int(x / max_distance * 119), 120 + int(y / max_distance * 119))
                lcd.set_at(point, pygame.Color(255, 255, 255))
    pygame.display.update()


scan_data = [0]*360

try:
    print(lidar.info)
    for scan in lidar.iter_scans():
        for (_, angle, distance) in scan:
            scan_data[min([359, floor(angle)])] = distance
        process_data(scan_data)
except KeyboardInterrupt:
    print('Stoping.')
lidar.stop()
lidar.disconnect()
