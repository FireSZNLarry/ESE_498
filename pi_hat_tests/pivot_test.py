import math
import time
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

def Servo_Motor_Initialization():
   i2c_bus = busio.I2C(SCL, SDA)
   pca = PCA9685(i2c_bus)
   pca.frequency = 100
   return pca

def Motor_Start(pca):
   x = input("Press and hold EZ button. Once the LED turns red, immediately release the button. After the LED blinks red once, press 'ENTER' on the keyboard.")
   Motor_Speed(pca, 0.15)
   time.sleep(2)
   y = input("If the LED just blinked TWICE, then press 'ENTER' on the keyboard.")
   Motor_Speed(pca, -0.15)
   time.sleep(2)
   z = input("Now the LED should be in solid green, indicating the initialization is complete. Press 'ENTER' on the keyboard to proceed")

def Motor_Speed(pca, percent):
   # converts a -1 to 1 value to 16-bit duty cycle
   speed = int((percent + 1) * 3277) + int(pca.channels[15].duty_cycle * 0.15)
   pca.channels[15].duty_cycle = speed

# initialization
pca = Servo_Motor_Initialization()
Motor_Start(pca)

# Motor control
def Move_Forward(pca):
   Motor_Speed(pca, 0.15)  # forward
   time.sleep(3)
   Motor_Speed(pca, 0)     # stop/neutral position

def Move_Reverse(pca):
   Motor_Speed(pca, -0.15)  # reverse
   time.sleep(3)
   Motor_Speed(pca, 0)      # stop/neutral position

def Turn(pca, angle):
   channel_num = 14  # example channel, modify based on your setup
   servo_motor = servo.Servo(pca.channels[channel_num])
   servo_motor.angle = angle
   time.sleep(1)

# Example movements
Move_Forward(pca)
time.sleep(2)
Move_Reverse(pca)
time.sleep(2)
Turn(pca, 90)  # adjust the angle as needed
