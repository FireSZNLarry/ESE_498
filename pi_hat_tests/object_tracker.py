#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np
from std_msgs.msg import String
import busio
from board import SCL, SDA
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

class ObjectTracker:
    def __init__(self):
        rospy.init_node('object_tracker_node', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback)

        # Motor and Servo setup
        i2c = busio.I2C(SCL, SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 100
        self.servo = servo.Servo(self.pca.channels[1])
        self.target_angle = 90

        rospy.spin()

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except cv_bridge.CvBridgeError as e:
            print(e)
        
        blue_hsv_min = np.array([100, 150, 0])
        blue_hsv_max = np.array([140, 255, 255])
        
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_image, blue_hsv_min, blue_hsv_max)
        conts, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if conts:
            largest_cont = max(conts, key=cv2.contourArea)
            M = cv2.moments(largest_cont)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                if cx < 140:
                    self.target_angle = 135  # Turn Left
                elif cx > 180:
                    self.target_angle = 45  # Turn Right
                else:
                    self.target_angle = 90  # Forward
                self.update_motor()
        else:
            self.stop_motor()

    def update_motor(self):
        self.pca.channels[15].duty_cycle = 0x7FFF  # Medium speed
        self.servo.angle = self.target_angle

    def stop_motor(self):
        self.pca.channels[15].duty_cycle = 0

if __name__ == '__main__':
    try:
        ObjectTracker()
    except rospy.ROSInterruptException:
        pass
