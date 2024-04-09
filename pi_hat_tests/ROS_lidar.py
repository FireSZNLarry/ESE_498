#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from adafruit_rplidar import RPLidar
import numpy as np

# Initializing the LIDAR
PORT_NAME = '/dev/ttyUSB0'
lidar = RPLidar(None, PORT_NAME, timeout=3)

def lidar_scan_publisher():
    # Initialize the node
    rospy.init_node('lidar_scan_publisher', anonymous=True)
    # Create a publisher object for publishing messages on the "scan" topic
    pub = rospy.Publisher('scan', LaserScan, queue_size=10)
    rate = rospy.Rate(10)  # 10hz
    
    while not rospy.is_shutdown():
        scan_data = [0]*360
        try:
            print(lidar.info())
            scans = next(lidar.iter_scans())
            for (_, angle, distance) in scans:
                angle = np.floor(angle)
                if angle < 360:
                    scan_data[int(angle)] = distance / 1000.0  # Convert to meters
                
            current_time = rospy.Time.now()
            # Populate LaserScan message
            scan = LaserScan()
            scan.header.stamp = current_time
            scan.header.frame_id = 'laser_frame'
            scan.angle_min = 0
            scan.angle_max = 2 * np.pi
            scan.angle_increment = np.pi / 180.0
            scan.time_increment = (1 / 10) / (360)
            scan.range_min = 0.2
            scan.range_max = 6.0
            
            scan.ranges = scan_data
            pub.publish(scan)  # Publish the scan data
            rate.sleep()
            
        except rospy.ROSInterruptException:
            print('Stopping.')
            lidar.stop()
            lidar.disconnect()

if __name__ == '__main__':
    lidar_scan_publisher()
