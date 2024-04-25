import time
import busio
import board
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
from adafruit_rplidar import RPLidar
import pygame

# Set up Pygame for LIDAR visualization
pygame.init()
lcd = pygame.display.set_mode((320, 240))
pygame.mouse.set_visible(False)
lcd.fill((0, 0, 0))
pygame.display.update()

# LIDAR setup
PORT_NAME = '/dev/ttyUSB0'
lidar = RPLidar(None, PORT_NAME)

# Motor and Servo setup
i2c_bus = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c_bus)
pca.frequency = 50
motor_channel = 15
steering_channel = 14
servo_steering = servo.Servo(pca.channels[steering_channel])
motor_servo = servo.ContinuousServo(pca.channels[motor_channel])

# Motor control functions
def control_motor(speed):
    if speed == 0:
        motor_servo.throttle = 0
    else:
        motor_servo.throttle = speed

def control_steering(angle):
    servo_steering.angle = angle

# Parameters
SAFE_DISTANCE = 500  # in millimeters
TURN_DISTANCE = 300
TURN_ANGLE = 30  # Degree to turn at obstacles

# Helper functions
def scale_lidar_distance(distance, max_distance=4000):
    return min(distance, max_distance) / max_distance

try:
    print("Starting object avoidance")
    while True:
        scan_data = [0]*360

        # Gather data
        for scan in lidar.iter_scans():
            for (_, angle, distance) in scan:
                scan_data[min([359, int(angle)])] = distance

        forward_distance = scan_data[0]  # direct front

        # Object avoidance logic
        if forward_distance < TURN_DISTANCE:
            # Too close, turn right sharply
            control_motor(-0.2)  # Reverse a bit
            control_steering(90 + TURN_ANGLE)  # Turn right
        elif forward_distance < SAFE_DISTANCE:
            # Close enough to worry, turn slightly
            control_motor(0.1)  # Forward slow
            control_steering(90 - TURN_ANGLE)  # Turn left
        else:
            # Path is clear, move faster
            control_motor(0.5)  # Forward normal speed
            control_steering(90)  # Straight ahead

        time.sleep(1)  # Adjust for real conditions

        # Update pygame display with LIDAR data
        lcd.fill((0, 0, 0))
        for angle in range(360):
            distance = scan_data[angle]
            if distance > 0:
                x = cos(math.radians(angle)) * distance
                y = sin(math.radians(angle)) * distance
                lcd.set_at((160 + int(x), 120 + int(y)), pygame.Color(255, 255, 255))
        pygame.display.update()

except KeyboardInterrupt:
    print('Stopping.')

finally:
    lidar.stop()
    lidar.disconnect()
    pca.deinit()
    pygame.quit()
    print("Shutdown complete.")
