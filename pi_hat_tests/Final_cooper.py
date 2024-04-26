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

CAMERA_DEVICE_ID = 0
IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240
fps = 0

color = input('What object do you want to find? Options are: ibuprofen, nyquil.\n')
if color == 'nyquil':
    hsv_min = np.array((70, 80, 80))
    hsv_max = np.array((100, 255, 255))
    print('nyquil color')
else:
    hsv_min = np.array((70, 80, 80))
    hsv_max = np.array((100, 255, 255))
    print('ibuprofen color')

colors = []
i = 0

i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 100
channel_num = 14
servo7 = servo.Servo(pca.channels[channel_num])

def isset(v):
    try:
        type (eval(v))
    except:
        return 0
    else:
        return 1

def hsv2rgb(h, s, v):
    h = float(h) * 2
    s = float(s) / 255
    v = float(v) / 255
    h60 = h / 60.0
    h60f = math.floor(h60)
    hi = int(h60f) % 6
    f = h60 - h60f
    p = v * (1 - s)
    q = v * (1 - f * s)
    t = v * (1 - (1 - f) * s)
    r, g, b = 0, 0, 0
    if hi == 0: r, g, b = v, t, p
    elif hi == 1: r, g, b = q, v, p
    elif hi == 2: r, g, b = p, v, t
    elif hi == 3: r, g, b = p, q, v
    elif hi == 4: r, g, b = t, p, v
    elif hi == 5: r, g, b = v, p, q
    r, g, b = int(r * 255), int(g * 255), int(b * 255)
    return (r, g, b)

def rgb2hsv(r, g, b):
    r, g, b = r/255.0, g/255.0, b/255.0
    mx = max(r, g, b)
    mn = min(r, g, b)
    df = mx-mn
    if mx == mn:
        h = 0
    elif mx == r:
        h = (60 * ((g-b)/df) + 360) % 360
    elif mx == g:
        h = (60 * ((b-r)/df) + 120) % 360
    elif mx == b:
        h = (60 * ((r-g)/df) + 240) % 360
    if mx == 0:
        s = 0
    else:
        s = df/mx
    v = mx
    h = int(h / 2)
    s = int(s * 255)
    v = int(v * 255)
    return (h, s, v)

def Motor_Speed(pca,percent):
    speed = ((percent) * 3277) + 65535 * 0.15
    pca.channels[15].duty_cycle = math.floor(speed)

if __name__ == "__main__":
    try:
        cap = cv2.VideoCapture(CAMERA_DEVICE_ID)
        cap.set(3, IMAGE_WIDTH)
        cap.set(4, IMAGE_HEIGHT)
        while True:
            _, frame = cap.read()
            frame = cv2.blur(frame,(3,3))
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            if colors:
                minh = min(c[0] for c in colors)
                mins = min(c[1] for c in colors)
                minv = min(c[2] for c in colors)
                maxh = max(c[0] for c in colors)
                maxs = max(c[1] for c in colors)
                maxv = max(c[2] for c in colors)
                hsv_min = np.array((minh, mins, minv))
                hsv_max = np.array((maxh, maxs, maxv))
            thresh = cv2.inRange(hsv, hsv_min, hsv_max)
            thresh2 = thresh.copy()
            (major_ver, minor_ver, subminor_ver) = (cv2.__version__).split('.')
            if major_ver == "2" or major_ver == "3":
                _, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            else:
                contours, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            max_area = 0
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > max_area:
                    max_area = area
                    best_cnt = cnt
            if isset('best_cnt'):
                M = cv2.moments(best_cnt)
                cx,cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
                if cx<120:
                    if cx>10:
                        print("right")
                        Motor_Speed(pca, 0.15) 
                        servo7.angle = 60
                        time.sleep(0.1)
                    else:
                        print("none")
                        Motor_Speed(pca, 0.15)
                        servo7.angle = 60
                        time.sleep(0.1)
                elif cx>120:
                    if cx<310:
                        if cx<220:
                            print("center")
                            Motor_Speed(pca, 0)  
                            servo7.angle = 95
                            time.sleep(0.1)
                        else:
                            print("left")
                            Motor_Speed(pca, 0.15)
                            servo7.angle = 120
                            time.sleep(0.1)
                    else:
                        print("none")
                        Motor_Speed(pca, 0.15)
                        servo7.angle = 60
                        time.sleep(0.1)
            else:
                print("DNR")
                Motor_Speed(pca, 0.15)
                servo7.angle = 60
                time.sleep(0.1)
    finally:
        cv2.destroyAllWindows()
        cap.release()
