#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading

class WallNavigator:
    def __init__(self):
        rospy.init_node('wall_navigator', anonymous=True)
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.handle_lidar_data)
        self.camera_sub = rospy.Subscriber('/camera/image_raw', Image, self.handle_camera_data)
        self.bridge = CvBridge()
        self.turn_angle = 77

    def handle_lidar_data(self, data):
        # Process LIDAR data
        rospy.loginfo("Received LIDAR data")

    def handle_camera_data(self, data):
        rospy.loginfo("Received camera frame")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
            self.process_image(cv_image)
        except Exception as e:
            rospy.logerr('Failed to convert image: %s', str(e))

    def process_image(self, image):
        # Process image data (placeholder for your logic)
        rospy.loginfo("Processing image")

def main():
    navigator = WallNavigator()
    rospy.spin()

if __name__ == '__main__':
    main()
