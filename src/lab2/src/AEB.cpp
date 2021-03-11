#include <ros/ros.h>

// Publish to a topic with this message type
#include <ackermann_msgs/AckermannDriveStamped.h>
// AckermannDriveStamped messages include this message type
#include <ackermann_msgs/AckermannDrive.h>

// Subscribe to a topic with this message type
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>

#include "std_msgs/String.h"
#include <string>
#include <cmath>

class AEBWalker {
private:
    // A ROS node
    ros::NodeHandle n;

    // car parameters
    double max_speed, max_steering_angle, car_width, car_length;

    // Publish drive data
    ros::Publisher drive_pub;

    //Listens for Laser Scan
    ros::Subscriber scan_sub;

    // previous desired steering angle
    double prev_angle=0.0;

    double minimunTTC = .22;

public:
    AEBWalker() {
        // Initialize the node handle
        n = ros::NodeHandle("~");

        // get topic names
        std::string drive_topic, scan_topic;
        // std::string testString;
        // testString = "THISISATEST";
        // ROS_INFO("TESTING");
        // ROS_INFO("%s",testString.c_str());
        n.getParam("AEB_drive_topic", drive_topic);
        n.getParam("scan_topic", scan_topic);
        // ROS_INFO("%s",drive_topic.c_str());
        // get car parameters
        n.getParam("max_speed", max_speed);
        n.getParam("max_steering_angle", max_steering_angle);
        n.getParam("width", car_width);
        n.getParam("wheelbase", car_length);
        // Make a publisher for drive messages
        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 10);
        
        //SHOULDNT NEED THE FOLLOWING CODE - Kept as backup
        // Start a subscriber to listen to odom messages
        //odom_sub = n.subscribe(odom_topic, 1, &AEBWalker::odom_callback, this);

        scan_sub = n.subscribe(scan_topic, 1, &AEBWalker::scan_callback, this);
    }
    //only looks for single point in front of car. can be limited by 
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg){
        float angleMin = msg->angle_min;
        float angleMax = msg->angle_max;
        float angleIncrement = msg->angle_increment;
        int index = static_cast<int>(angleMin*-1 / angleIncrement);
        float distance = msg->ranges.at(index);
        float timeToCollision = distance/max_speed;
        ackermann_msgs::AckermannDriveStamped drive_st_msg;
        ackermann_msgs::AckermannDrive drive_msg;
        drive_msg.steering_angle = 0;
        if(timeToCollision<0.5){
            drive_msg.speed =0;
        }else{
            drive_msg.speed =max_speed;
        }
        drive_st_msg.drive = drive_msg;
        drive_pub.publish(drive_st_msg);

    }
    //beginning of more advanced AEB
    /*void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg){
        float angleMin = msg->angle_min;
        float angleMax = msg->angle_max;
        float angleIncrement = msg->angle_increment;
        //float thisValue = msg->ranges.at(index);
        for(int index =0; index<static_cast<int>(msg->ranges.size());index++){
            float currentAngle = angleMin + angleIncrement * index;
            float distance = msg->ranges.at(index);
            float velocity = max_speed * std::cos(currentAngle);
            float timeToCollision =-1;
            if(velocity>0){
                timeToCollision = distance/velocity;
            }
            ROS_INFO("The time to collision: %s",(std::to_string(timeToCollision)).c_str());
        }
    }*/

}; // end of class definition

int main(int argc, char ** argv) {
    ros::init(argc, argv, "AEB_drive");
    AEBWalker rw;
    ros::spin();
    return 0;
}