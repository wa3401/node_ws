#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan

class points():


    def __init__(self):
        self.pub1 = rospy.Publisher('/closest_point', Float64, queue_size=10)
        self.pub2 = rospy.Publisher('/farthest_point', Float64, queue_size=10)
        rospy.init_node('points', anonymous=False)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.callback)
        

    def callback(self, data):
        self.pub1.publish(data.range_min)
        self.pub2.publish(data.range_max)

if __name__ == '__main__':
    points()
    rospy.spin()
