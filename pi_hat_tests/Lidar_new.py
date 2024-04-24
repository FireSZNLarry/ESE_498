import math
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

# Function to update motor speed
def update_motor_speed(speed):
    # Maps speed range -1.0 to 1.0 to appropriate PWM values
    pwm_value = int((speed * 32767) + 32767)
    motor.duty_cycle = pwm_value

# Function to update steering angle
def update_steering_angle(angle):
    servo_steering.angle = angle

# Helper function to scale LIDAR distance data
def scale_lidar_distance(distance, max_distance=4000):
    return min(distance, max_distance) / max_distance

# Control parameters
desired_distance_from_wall = 1000  # in millimeters
distance_tolerance = 100  # mm tolerance for distance maintenance
max_speed = 0.5
turn_sensitivity = 10  # Higher values mean more sensitive turning

# Main loop
try:
    scan_data = [0]*360
    for scan in lidar.iter_scans():
        for (_, angle, distance) in scan:
            scan_data[min([359, floor(angle)])] = distance
        # Assume wall is always to the right for simplicity (90 degree angle)
        distance_to_wall = scan_data[90] or desired_distance_from_wall
        error = desired_distance_from_wall - distance_to_wall

        # Motor speed control based on proximity to the wall
        if abs(error) > distance_tolerance:
            speed = max_speed * (error / desired_distance_from_wall)
            speed = max(-max_speed, min(max_speed, speed))  # Limit speed range
        else:
            speed = 0
        update_motor_speed(speed)

        # Steering control based on error
        steering_angle = 90 + (error / distance_tolerance) * turn_sensitivity
        update_steering_angle(steering_angle)

        # Update pygame display with LIDAR data
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
