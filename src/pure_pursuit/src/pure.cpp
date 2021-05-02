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
#include <visualization_msgs/Marker.h>

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
    //Initializes Publishers and Subscirbers
    ros::NodeHandle n;

    ros::Publisher drive_pub;

    ros::Publisher waypoint_pub;

    ros::Subscriber pose_sub;

    std::vector<f110::WayPoint> waypoints;

    //Object levek variables
    f110::WayPoint bestPoint;
    double bestDif;
    f110::WayPoint currWay;
    bool firstPoint = true;
    int num_waypoints;

    size_t uniqueMarkerId;
    size_t lastBestIdx;
    tf2_ros::TransformListener tfListener;
    tf2_ros::Buffer tfBuffer;
    bool visualized;


public:

    
    //Class level variables
    float L = 1.0;
    float wheelbase;
    //Constructor
    Pure() :
        //Subscribers and publishers
        n(ros::NodeHandle()),
        pose_sub(n.subscribe("gt_pose", 5, &Pure::pose_callback, this)),
        drive_pub(n.advertise<ackermann_msgs::AckermannDriveStamped>("/pure_drive", 1)),
        waypoint_pub(n.advertise<visualization_msgs::Marker>("waypoint_markers", 100)),

        //Initializes variables
        visualized(false),
        lastBestIdx(0),
        tfListener(tfBuffer)
    {
        n = ros::NodeHandle("~");

        //Gets vaiables from params.yaml file
        std::string drive_topic, scan_topic, pose_topic;
        n.getParam("pure_drive_topic", drive_topic);
        n.getParam("scan_topic", scan_topic);
        pose_topic = "/gt_pose";
        n.getParam("wheelbase", wheelbase);
        num_waypoints = 700;

        //CSV reader for the waypoint data
        f110::CSVReader reader("/home/williamanderson/catkin_ws/src/pure_pursuit/src/newPoints.csv");
        //Sets all waypoints into the vector
        waypoints = reader.getData(num_waypoints);
        //Wisualizes waypoints
        visualize_waypoint_data();
        //Waits for 1 second
        ros::Duration(1.0).sleep();
    }

    void pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg){
        ackermann_msgs::AckermannDriveStamped drive_st_msg;

        //Initializes best point to be a WayPoint
        bestPoint = f110::WayPoint(msg);

        //Tansforms waypoints to car frame
        const auto transformedWayPoints = transform(waypoints, bestPoint, tfBuffer, tfListener);

        //Gets the best waypoint to track currently
        const auto goalWayPointIdx = f110::get_best_track_point_index(transformedWayPoints, L, lastBestIdx);

        //Finds the transformation from the map to base_link
        geometry_msgs::TransformStamped mapToBaseLink = tfBuffer.lookupTransform("base_link", "map", ros::Time(0));

        //Sets all of the paramaters for the goalWayPoint
        geometry_msgs::Pose goalWayPoint;
        goalWayPoint.position.x = waypoints[goalWayPointIdx].x;
        goalWayPoint.position.y = waypoints[goalWayPointIdx].y;
        goalWayPoint.position.z = 0;
        goalWayPoint.orientation.x = 0;
        goalWayPoint.orientation.y = 0;
        goalWayPoint.orientation.z = 0;
        goalWayPoint.orientation.w = 0;
        //Transforms the goal waypoint
        tf2::doTransform(goalWayPoint, goalWayPoint, mapToBaseLink);

        //Visualizes the goal waypoint
        add_way_point_visualization(goalWayPoint, "base_link", 1.0, 0.0, 0.0, 0.3, 0.2, 0.2, 0.2);
        //Sets steering angle based ont he goalWayPoint
        double steeringAngle = 2*(goalWayPoint.position.y)/(L*L);

        //Sets the stamped drive message
        drive_st_msg.header.stamp = ros::Time::now();
        drive_st_msg.header.frame_id = "base_link";

        drive_st_msg.drive.steering_angle = steeringAngle;
        //Sets speed based on steering angle
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

    //Function to visualize a waypoint
    void add_way_point_visualization(const f110::WayPoint& way_point, const std::string& frame_id,
            double r, double g, double b, double transparency = 0.5, double scale_x=0.1, double scale_y=0.1,
            double scale_z=0.1)
    {
        visualization_msgs::Marker way_point_marker;
        way_point_marker.header.frame_id = frame_id;
        way_point_marker.header.stamp = ros::Time();
        way_point_marker.ns = "pure_pursuit";
        way_point_marker.id = uniqueMarkerId;
        way_point_marker.type = visualization_msgs::Marker::SPHERE;
        way_point_marker.action = visualization_msgs::Marker::ADD;
        way_point_marker.pose.position.x = way_point.x;
        way_point_marker.pose.position.y = way_point.y;
        way_point_marker.pose.position.z = 0;
        way_point_marker.pose.orientation.x = 0.0;
        way_point_marker.pose.orientation.y = 0.0;
        way_point_marker.pose.orientation.z = 0.0;
        way_point_marker.pose.orientation.w = 1.0;
        way_point_marker.scale.x = scale_x;
        way_point_marker.scale.y = scale_y;
        way_point_marker.scale.z = scale_z;
        way_point_marker.color.a = transparency;
        way_point_marker.color.r = r;
        way_point_marker.color.g = g;
        way_point_marker.color.b = b;
        waypoint_pub.publish(way_point_marker);
        uniqueMarkerId++;
    }

    //Visualizes every 5th waypoint in the waypoint
    void visualize_waypoint_data(){
        const size_t inc = waypoints.size()/5;
        for(size_t i = 0, j = 0; i <waypoints.size(); i + i + inc, j++){
            add_way_point_visualization(waypoints[i], "map", 0.0, 0.0, 1.0, 0.5);
        }
    }

    
};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "pure_pursuit_node");
    Pure rw;
    ros::spin();
    return 0;
}