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
import motorControl as momo

i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 100
#channel_num = 14
momo.Motor_Speed(pca,0.175)

os.putenv('SDL_FBDEV', '/dev/fb1')
pygame.init()

PORT_NAME = '/dev/ttyUSB0'
lidar = RPLidar(None, PORT_NAME, timeout=3)

#i2c_bus = busio.I2C(SCL, SDA)
#pca = PCA9685(i2c_bus)
#pca.frequency = 100
steering_channel = 14
motor_channel = 15

servo_steering = servo.Servo(pca.channels[steering_channel])
#motor = pca.channels[motor_channel]

def update_steering_angle(angle):
    servo_steering.angle = angle

def scale_lidar_distance(distance, max_distance=3000):
    return min(distance, max_distance) / max_distance

def main():
    try:
        scan_data = [0]*360
        while True:
            for scan in lidar.iter_scans():
                for (_, angle, distance) in scan:
                    angle = int(angle)
                    if 80 <= angle < 200:
                        scan_data[angle] = distance
                        print(distance)
                        if distance < 1000:
                          update_steering_angle(70)
                          time.sleep(0.1)
                          if distance < 700:
                              momo.Motor_Speed(pca,0)
                              
                for angle in range(360):
                    distance = scan_data[angle]
                    if distance:
                        scaled_distance = scale_lidar_distance(distance)
                        radians = angle * pi / 180
                        x = scaled_distance * cos(radians) * 119
                        y = scaled_distance * sin(radians) * 119
                        point = (160 + int(x), 120 + int(y))

    except KeyboardInterrupt:
        print('Stopping.')
    finally:
        lidar.stop()
        lidar.disconnect()
        pygame.quit()

if __name__ == "__main__":
    main()
