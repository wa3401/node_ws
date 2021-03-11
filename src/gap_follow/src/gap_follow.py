#!/usr/bin/env python3
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class reactive_follow_gap:

    global prev_drive
    global prev_drive_st_msg

    def __init__(self):
        #Topics & Subscriptions,Publishers
        lidarscan_topic = rospy.get_param("scan_topic")
        drive_topic = rospy.get_param("gap_drive_topic")

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)
    
    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        DISCREPENCY_DIST = 0.4
        proc_ranges = []
        for i in range(ranges):
            if ranges[i] - ranges[i+1] <  DISCREPENCY_DIST and ranges[i] < 3:
                proc_ranges.append(ranges[i])
        return proc_ranges

    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	Naive: Choose the furthest point within ranges and go there
        """
        maxLenIdx = 0
        maxLen = 0
        for i in range(ranges):
            if ranges[i] > maxLen:
                maxLen = ranges[i]
                maxLenIdx = i
        return maxLenIdx

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges
        proc_ranges = self.preprocess_lidar(ranges)
        drive_st_msg = AckermannDriveStamped()
        drive_msg = AckermannDrive()
        prev_drive = AckermannDrive()
        prev_drive_st_msg = AckermannDriveStamped()
        

        #Find closest point to LiDAR
        minPointIdx = 0
        for i in range(proc_ranges) :
            if proc_ranges[i] < proc_ranges[minPointIdx] :
                minPointIdx = i

        carLength = rospy.get_param("wheelbase")
        carWidth = rospy.get_param("width")
        avoidRadius = math.sqrt(math.pow(carLength, 2) + math.pow(carWidth, 2) + 1)
        angleToConsider = math.atan(avoidRadius / proc_ranges[minPointIdx])
        numPointsReplace = math.ceil(angleToConsider / data.angle_increment)

        for i in range(numPointsReplace) :
            proc_ranges[minPointIdx - i] = 0
            proc_ranges[minPointIdx + i] = 0
        
        maxGap = 0
        count = 0
        idxStartGap = 0
        idxEndGap = 0
        for i in range(proc_ranges) :
            if proc_ranges[i] != 0 :
                if count == 0:
                    idxStartGap = i
                count += 1
            else :
                if count > maxGap :
                    maxGap = count
                    idxEndGap = i - 1
                count = 0

        bestPoint = self.find_best_point(idxStartGap, idxEndGap, proc_ranges)
        finalSteeringAngle = data.angle_min + data.angle_increment * bestPoint
        drive_msg.steering_angle = finalSteeringAngle
        if abs(drive_msg.steering_angle < 10):
            drive_msg.speed = 5
        elif abs(drive_msg.steering_angle < 20):
            drive_msg.speed = 2
        else:
            drive_msg.speed = 1

        prev_drive = drive_msg
        drive_st_msg.drive = drive_msg
        prev_drive_st_msg.drive = prev_drive

        if bestPoint > 3: 
            self.drive_pub.publish(prev_drive_st_msg)
        else :
            self.drive_pub.publish(drive_st_msg)



        #Eliminate all points inside 'bubble' (set them to zero) 

        #Find max length gap 

        #Find the best point in the gap 

        #Publish Drive message

def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)