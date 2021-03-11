#!/usr/bin/env python3

import rospy
from beginner_tutorials.msg import scan_range
from sensor_msgs.msg import LaserScan

class lidar_processing():


    def __init__(self):
        self.pub1 = rospy.Publisher('/scan_range', scan_range, queue_size=10)
        rospy.init_node('lidar_processing', anonymous=False)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.callback)

    def callback(self, data):
        mess = scan_range
        mess.min = data.range_min
        mess.max = data.range_max
        self.pub1.publish(mess)

if __name__ == '__main__':
    lidar_processing()
    rospy.spin()
