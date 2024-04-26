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
import threading
import queue
import motorControl as momo

i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 100
channel_num = 14
servo7 = servo.Servo(pca.channels[channel_num])
os.putenv('SDL_FBDEV', '/dev/fb1')
pygame.init()
PORT_NAME = '/dev/ttyUSB0'
lidar = RPLidar(None, PORT_NAME, timeout=3)
steering_channel = 14
motor_channel = 15
servo_steering = servo.Servo(pca.channels[steering_channel])


momo.Motor_Speed(pca,0)
time.sleep(0.5)
momo.Motor_Speed(pca,-0.15)
update_steering_angle(70)
time.sleep(0.5)
momo.Motor_Speed(pca,0)
update_steering_angle(95)
time.sleep(0.5)
momo.Motor_Speed(pca,0.15)
update_steering_angle(130)
momo.Motor_Speed(pca,0)
time.sleep(0.5)
momo.Motor_Speed(pca,-0.15)
update_steering_angle(70)
time.sleep(0.5)
momo.Motor_Speed(pca,0)
update_steering_angle(95)
time.sleep(0.5)
momo.Motor_Speed(pca,0.15)
update_steering_angle(130)
momo.Motor_Speed(pca,0)
time.sleep(0.5)
momo.Motor_Speed(pca,-0.15)
update_steering_angle(70)
time.sleep(0.5)
momo.Motor_Speed(pca,0)
update_steering_angle(95)
time.sleep(0.5)
momo.Motor_Speed(pca,0.15)
update_steering_angle(130)

def update_steering_angle(angle):
    servo_steering.angle = angle

    

