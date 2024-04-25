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

# Set up pygame for LIDAR visualization
os.putenv('SDL_FBDEV', '/dev/fb1')
pygame.init()
lcd = pygame.display.set_mode((320, 240))
pygame.mouse.set_visible(False)
lcd.fill((0, 0, 0))
pygame.display.update()

# LIDAR setup
PORT_NAME = '/dev/ttyUSB0'
lidar = RPLidar(None, PORT_NAME, timeout=3)

# Motor and Servo setup
i2c_bus = busio.I2C(SCL, SDA)
pca = PCA9685(i2c_bus)
pca.frequency = 100
motor_channel = 15
steering_channel = 14

servo_steering = servo.Servo(pca.channels[steering_channel])
motor = pca.channels[motor_channel]

# Helper function to update motor speed
def update_motor_speed(speed):
    pwm_value = int((speed * 32767) + 32767)
    motor.duty_cycle = pwm_value

# Helper function to update steering angle
def update_steering_angle(angle):
    servo_steering.angle = angle

# Helper function to scale LIDAR distance data
def scale_lidar_distance(distance, max_distance=4000):
    return min(distance, max_distance) / max_distance

# Control parameters for centering in a room or corridor
desired_distance_from_wall = 1524  # desired distance from the wall is 5 feet (1524 mm)
distance_tolerance = 100  # mm tolerance for distance maintenance
max_speed = 0.5
turn_sensitivity = 10  # Higher sensitivity in turning

def main():
    try:
        scan_data = [0]*360
        while True:
            for scan in lidar.iter_scans():
                for (_, angle, distance) in scan:
                    angle = int(angle)
                    if 0 <= angle < 360:
                        scan_data[angle] = distance

                # Calculate errors for both right and left (90 degrees and 270 degrees respectively)
                right_distance = scan_data[90] or desired_distance_from_wall
                left_distance = scan_data[270] or desired_distance_from_wall
                right_error = desired_distance_from_wall - right_distance
                left_error = desired_distance_from_wall - left_distance

                # Calculate motor speed and steering angle based on average error
                average_error = (right_error + left_error) / 2
                if abs(average_error) > distance_tolerance:
                    speed = (max_speed * average_error / desired_distance_from_wall)
                    speed = max(-max_speed, min(max_speed, speed))
                else:
                    speed = 0
                update_motor_speed(speed)

                # Adjust steering based on error difference
                error_difference = left_error - right_error
                steering_angle = 90 + (error_difference / distance_tolerance) * turn_sensitivity
                update_steering_angle(steering_angle-20)

                # Update visualization
                lcd.fill((0, 0, 0))
                for angle in range(360):
                    distance = scan_data[angle]
                    if distance:
                        scaled_distance = scale_lidar_distance(distance)
                        radians = angle * pi / 180
                        x = scaled_distance * cos(radians) * 119
                        y = scaled_distance * sin(radians) * 119
                        point = (160 + int(x), 120 + int(y))
                        lcd.set_at(point, pygame.Color(255, 255, 255))
                pygame.display.update()

    except KeyboardInterrupt:
        print('Stopping.')
    finally:
        lidar.stop()
        lidar.disconnect()
        pygame.quit()

if __name__ == "__main__":
    main()
