#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan

def callback(data):
    rospy.loginfo("I heard %s", data.data)

def scan_info():
    rospy.init_node('scan info', anonymous=True)

    rospy.Subscriber("scan", sensor_msgs.LaserScan, callback)

    rospy.spin()

if __name__ == '__main__':
    scan_info()