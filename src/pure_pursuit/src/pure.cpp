#include <ros/ros.h>

// Publish to a topic with this message type
#include <ackermann_msgs/AckermannDriveStamped.h>
// AckermannDriveStamped messages include this message type
#include <ackermann_msgs/AckermannDrive.h>

#include <geometry_msgs/PoseStamped.h>

// Subscribe to a topic with this message type
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <tf2_msgs/TFMessage.h>

#include "std_msgs/String.h"
#include <string>
#include <cmath>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>

#include "pure_pursuit/csv_reader.h"
#include "pure_pursuit/pure_pursuit.h"
#include "pure_pursuit/types.h"

using namespace std;

class Pure {

private:
    ros::NodeHandle n;

    ros::Publisher drive_pub;

    ros::Subscriber pose_sub;

    std::vector<f110::WayPoint> waypoints;

    f110::WayPoint bestPoint;
    double bestDif;
    f110::WayPoint currWay;
    bool firstPoint = true;
    int num_waypoints;

    size_t uniqueMarkerId;
    size_t lastBestIdx;
    tf2_ros::TransformListener tfListener;
    tf2_ros::Buffer tfBuffer;

public:

    

    float L = 1.0;
    float wheelbase;
    
    Pure() :
        n(ros::NodeHandle()),
        pose_sub(n.subscribe("gt_pose", 5, &Pure::pose_callback, this)),
        drive_pub(n.advertise<ackermann_msgs::AckermannDriveStamped>("/pure_drive", 1)),
        lastBestIdx(0),
        tfListener(tfBuffer)
    {
        n = ros::NodeHandle("~");

        std::string drive_topic, scan_topic, pose_topic;
        n.getParam("pure_drive_topic", drive_topic);
        n.getParam("scan_topic", scan_topic);
        pose_topic = "/gt_pose";
        n.getParam("wheelbase", wheelbase);
        num_waypoints = 700;

        f110::CSVReader reader("/home/williamanderson/catkin_ws/src/pure_pursuit/src/newPoints.csv");

        waypoints = reader.getData(num_waypoints);

        ros::Duration(1.0).sleep();
    }

    void pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg){
        ackermann_msgs::AckermannDriveStamped drive_st_msg;
        ackermann_msgs::AckermannDrive drive_msg;

        bestPoint = f110::WayPoint(msg);

        const auto transformedWayPoints = transform(waypoints, bestPoint, tfBuffer, tfListener);

        const auto goalWayPointIdx = f110::get_best_track_point_index(transformedWayPoints, L, lastBestIdx);

        geometry_msgs::TransformStamped mapToBaseLink = tfBuffer.lookupTransform("base_link", "map", ros::Time(0));

        geometry_msgs::Pose goalWayPoint;
        goalWayPoint.position.x = waypoints[goalWayPointIdx].x;
        goalWayPoint.position.y = waypoints[goalWayPointIdx].y;
        goalWayPoint.position.z = 0;
        goalWayPoint.orientation.x = 0;
        goalWayPoint.orientation.y = 0;
        goalWayPoint.orientation.z = 0;
        goalWayPoint.orientation.w = 0;
        tf2::doTransform(goalWayPoint, goalWayPoint, mapToBaseLink);

        double steeringAngle = 2*(goalWayPoint.position.y)/(L*L);

        drive_st_msg.header.stamp = ros::Time::now();
        drive_st_msg.header.frame_id = "base_link";

        drive_st_msg.drive.steering_angle = steeringAngle;
        
        if(std::abs(steeringAngle)<.1){
            drive_st_msg.drive.speed = 6;
        }
        else if(std::abs(steeringAngle)<.2){
            drive_st_msg.drive.speed = 4;
        }
        else{
           drive_st_msg.drive.speed = 3;
        }
        drive_pub.publish(drive_st_msg);
    }

    float findDistance(float p1x, float p1y, float p2x, float p2y){
        float dx = p1x - p2x;
        float dy = p1y - p2y;
        return std::hypot(dx, dy);
    }

    
};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "pure_pursuit_node");
    Pure rw;
    ros::spin();
    return 0;
}