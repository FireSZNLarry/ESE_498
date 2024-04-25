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



import cv2
import numpy as np
import time
from time import sleep
import math
from board import SCL,SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

import os
from math import cos, sin, pi, floor
import pygame
from adafruit_rplidar import RPLidar

#from picamera import PiCamera
import threading
import queue



i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 100
channel_num = 14
servo7 = servo.Servo(pca.channels[channel_num])


servo7.angle = 70
time.sleep(2)

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

#pylint: disable=redefined-outer-name,global-statement
def process_data(data):
    global max_distance
    lcd.fill((0,0,0))
    for angle in range(60):
        distance = data[angle]
        if distance > 0:                  # ignore initially ungathered data points
            max_distance = max([min([5000, distance]), max_distance])
            radians = angle * pi / 180.0
            print('radians')
            print(radians)
            x = distance * cos(radians)
            print('x')
            print(x)
            if x<500:
                print('x is less than 500')
                servo7.angle = 70
                time.sleep(2)
            y = distance * sin(radians)
            print('y')
            print(y)
            if y<500:
                print('y is less than 500')
                servo7.angle = 70
                time.sleep(2)
            point = (160 + int(x / max_distance * 119), 120 + int(y / max_distance * 119))
            print('point')
            print(point)
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
