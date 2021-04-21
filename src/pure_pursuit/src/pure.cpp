#include <ros/ros.h>

// Publish to a topic with this message type
#include <ackermann_msgs/AckermannDriveStamped.h>
// AckermannDriveStamped messages include this message type
#include <ackermann_msgs/AckermannDrive.h>

#include <geometry_msgs/PoseStamped.h>

// Subscribe to a topic with this message type
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>

#include "std_msgs/String.h"
#include <string>
#include <cmath>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>

using namespace std;

class Pure {

    struct Waypoint {
        float X;
        float Y;
        float euler;
        float speed;
    };
private:
    ros::NodeHandle n;

    ros::Publisher drive_pub;

    ros::Subscriber scan_sub;

    ros::Subscriber pose_sub;

    std::vector<Waypoint> waypoints;

    Waypoint bestPoint;
    double bestDif;
    Waypoint currWay;
    bool firstPoint = true;

public:

    

    float L = 2.0;
    float wheelbase;
    
    Pure() {
        n = ros::NodeHandle("~");

        std::string drive_topic, scan_topic, pose_topic;
        n.getParam("AEB_drive_topic", drive_topic);
        n.getParam("scan_topic", scan_topic);
        n.getParam("pose_topic", pose_topic);
        n.getParam("wheelbase", wheelbase);

        string temp, line;
        float X, Y, euler, speed;

        ifstream waypointStream("points.csv");
        if(!waypointStream.is_open()){
            ROS_INFO("File failed to open");
        }

        while(getline(waypointStream, line)){
            stringstream ss(line);
            getline(ss, temp, ',');
            X = stof(temp);
            getline(ss, temp, ',');
            Y = stof(temp);
            getline(ss, temp, ',');
            euler = stof(temp);
            getline(ss, temp, ',');
            speed = stof(temp);
            Waypoint p;
            p.X = X;
            p.Y = Y;
            p.euler = euler;
            p.speed = speed;
            waypoints.push_back(p);
        }

        waypointStream.close();

        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 10);

        pose_sub = n.subscribe(pose_topic, 1, &Pure::pose_callback, this);

        
    }

    void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
        ackermann_msgs::AckermannDriveStamped drive_st_msg;
        ackermann_msgs::AckermannDrive drive_msg;

        bestPoint.X = 0;
        bestPoint.Y = 0;
        bestPoint.euler = 0;
        bestPoint.speed = 0;
        bestDif = std::numeric_limits<double>::infinity();
        float dif;

        if(firstPoint){
            currWay.X = msg->pose.position.x;
            currWay.Y = msg->pose.position.y;
            firstPoint = false;
        }

        if(std::abs(msg->pose.position.x - currWay.X) < 0.5 || std::abs(msg->pose.position.y - currWay.Y) < 0.5){
            for(Waypoint p : waypoints){
                dif = L - std::abs(findDistance(p.X, p.Y, msg->pose.position.x, msg->pose.position.y));
                if(dif < bestDif){
                   // ROS_INFO("Found a new waypoint: X: %s Y: %s", std::to_string(p.X), std::to_string(p.Y));
                    bestPoint.X = p.X;
                    bestPoint.Y = p.Y;
                    bestPoint.euler = p.euler;
                    bestPoint.speed = p.speed;
                }
            }
            currWay.X = bestPoint.X;
            currWay.Y = bestPoint.Y;
            currWay.euler = bestPoint.euler;
            currWay.speed = bestPoint.speed;
        }

        float distX = currWay.X - msg->pose.position.x;
        float distY = currWay.Y - msg->pose.position.y;

        float carFrameX = (distX - (distY * std::atan(msg->pose.orientation.w))) * std::acos(msg->pose.orientation.w);
        float carFrameY = (distX - (distY * std::atan(msg->pose.orientation.w))) * std::asin(msg->pose.orientation.w) + std::acos(msg->pose.orientation.w);

        float radius = std::pow(L, 2) / (2 * std::abs(carFrameY));
        float curve = 1 / radius;
        float speed_set = 3;

        float steer_angle = std::atan(curve * wheelbase);
        if(std::abs(steer_angle)<10){
            speed_set = 1.5;
        }
        else if(std::abs(steer_angle)<20){
            speed_set = 1;
        }
        else{
            speed_set = 0.5;
        }

        drive_msg.steering_angle = steer_angle;
        drive_msg.speed = speed_set;
        drive_st_msg.drive = drive_msg;
        drive_pub.publish(drive_st_msg);
    }

    float findDistance(float p1x, float p1y, float p2x, float p2y){
        float dx = p1x - p2x;
        float dy = p1y - p2y;
        return std::hypot(dx, dy);
    }

    
};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "AEB_drive");
    Pure rw;
    ros::spin();
    return 0;
}