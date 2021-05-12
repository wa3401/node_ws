#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <sensor_msgs/LaserScan.h>
#include <string>
#include <cmath>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include <random>
double lower_bound_range = -15;
double upper_bound_range = 15;
std::uniform_real_distribution<double> unif(lower_bound_range,upper_bound_range);
std::default_random_engine re;
class ReactiveMethod;
double distance(geometry_msgs::Point point1,geometry_msgs::Point point2);
double getAngle(geometry_msgs::Point point1,geometry_msgs::Point point2,geometry_msgs::Point point3){
    if(point1.x == point2.x && point1.y == point2.y){
        geometry_msgs::Point zero;
        zero.x=0;
        zero.y=0;
        if(point2.x ==0 && point2.y == 0){
            geometry_msgs::Point one;
            one.x = 1;
            one.y =0;
            return acos((pow(distance(one,zero),2)+pow(distance(zero,point3),2)-pow(distance(one,point3),2))/(2*distance(one,zero)*distance(zero,point3)));
        }
        return acos((pow(distance(zero,point2),2)+pow(distance(point2,point3),2)-pow(distance(zero,point3),2))/(2*distance(zero,point2)*distance(point2,point3)));
    }
    return acos((pow(distance(point1,point2),2)+pow(distance(point2,point3),2)-pow(distance(point1,point3),2))/(2*distance(point1,point2)*distance(point2,point3)));
}
int8_t getGridValueFromStandardXYCord(geometry_msgs::Point pointInXYFrame);
double distance(geometry_msgs::Point point1,geometry_msgs::Point point2){
    return std::sqrt(std::pow((point1.x - point2.x),2)+std::pow((point1.y-point2.y),2));
}
class RRTStarNode{
    public:
    RRTStarNode(double x, double y, RRTStarNode& closestNode);
    RRTStarNode(double x, double y);
    RRTStarNode* lastNode;
    geometry_msgs::Point coordinates;
    bool hasLastNode;
    double getTotalCost();
    double distanceToClosestNode();
    bool isAncestor(RRTStarNode otherNode);
};
double RRTStarNode::distanceToClosestNode(){
    if(hasLastNode){
        return distance(lastNode->coordinates,coordinates);
    }
    return 0;
}
double RRTStarNode::getTotalCost(){
    if(!hasLastNode){
        return 0;
    }
    else return distanceToClosestNode() + lastNode->getTotalCost();
}
RRTStarNode::RRTStarNode(double x, double y, RRTStarNode& closestNode){
    geometry_msgs::Point newPoint;
    newPoint.x = x;
    newPoint.y = y;
    coordinates = newPoint;
    lastNode = &closestNode;
    hasLastNode=true;
    
}
RRTStarNode::RRTStarNode(double x, double y){
    geometry_msgs::Point newPoint;
    newPoint.x = x;
    newPoint.y = y;
    coordinates = newPoint;
    lastNode = nullptr;
    hasLastNode =false;
}
bool RRTStarNode::isAncestor(RRTStarNode otherNode){
    RRTStarNode currentAncestor = *this;
    while(currentAncestor.hasLastNode){
        currentAncestor = *currentAncestor.lastNode;
        if(currentAncestor.coordinates == otherNode.coordinates){
            return true;
        }
    }
    return false;
}
class RRTStarPathTree{
    public:
    RRTStarPathTree(RRTStarNode& head);
    std::vector<RRTStarNode> listOfNodes;
    RRTStarNode* startNode;
    void reWireTree(int indexOfAddedNode);
    bool addNode(int& nodeAdded,ReactiveMethod& reactor);

};
void RRTStarPathTree::reWireTree(int nodeAdded){
    //segementation fault likely from infinite loop where it causes chain loop
    // need to consider ancestors before rewiring
    if(nodeAdded >0 && nodeAdded <listOfNodes.size())
    for(int i=1;i<static_cast<int>(listOfNodes.size());i++){
        if( i != nodeAdded && distance(listOfNodes.at(i).coordinates,listOfNodes.at(nodeAdded).coordinates)<3.0){
            if(listOfNodes.at(i).getTotalCost() > distance(listOfNodes.at(i).coordinates,listOfNodes.at(nodeAdded).coordinates) +listOfNodes.at(nodeAdded).getTotalCost()){
                if(!listOfNodes.at(i).isAncestor(listOfNodes.at(nodeAdded))){
                listOfNodes.at(i).lastNode = &listOfNodes.at(nodeAdded);
                }
            }
        }
    }
}

RRTStarPathTree::RRTStarPathTree(RRTStarNode& head){
    startNode = &head;
    listOfNodes.push_back(*startNode);
}
class ReactiveMethod {
private:
    // A ROS node
    ros::NodeHandle n;

    // car parameters
    double max_speed, max_steering_angle, car_width, car_length, max_acceleration,car_radius;

    // Publish drive data
    ros::Publisher drive_pub;

    //Listens for Laser Scan
    //ros::Subscriber scan_sub;

    ros::Subscriber map_sub;

    ros::Subscriber pos_sub;

    ros::Publisher vis_pub;
    double lastCarX=0;
    double lastCarY=0;
    //float DISCREPANCY_DISTANCE = .4; 
    float SCAN_WITHIN_ANGLE = ( 90 / 180.0) * M_PI; 
    double currentSpeed = 0;

    nav_msgs::OccupancyGrid grid;
    bool mapCreated = false;
    visualization_msgs::Marker dots;
    double avoidRadius = 0.5;

public:
double angleOfCar=0;

    ReactiveMethod() {
        n = ros::NodeHandle("~");
        std::string drive_topic, scan_topic;
        grid = nav_msgs::OccupancyGrid();
        n.getParam("pure_drive_topic", drive_topic);
        n.getParam("scan_topic", scan_topic);
        n.getParam("max_speed", max_speed);
        n.getParam("max_steering_angle", max_steering_angle);
        n.getParam("width", car_width);
        n.getParam("wheelbase", car_length);
        n.getParam("max_decel",max_acceleration);
        car_radius =sqrt(pow(car_width,2) +pow(car_length,2))/2;
        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 10);
        //scan_sub = n.subscribe(scan_topic, 1, &ReactiveMethod::scan_callback, this);
        map_sub = n.subscribe("/gmapping_map",1,&ReactiveMethod::map_callback,this);
        pos_sub = n.subscribe("/tf",1,&ReactiveMethod::pose_callback,this);
        vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

    }
    void testMap(){

         //ROS_INFO("%s",std::to_string(msg->angle_min).c_str());
        //  ADD IF CHECKS TO SEE IF NULL
        int height = grid.info.height;
        int width = grid.info.width;
        geometry_msgs::Point origin = grid.info.origin.position;
        float resolution = grid.info.resolution;
        dots.header.frame_id = "map";
        dots.ns = "test dots";
        dots.action = visualization_msgs::Marker::ADD;
        dots.pose.orientation.w = 1.0;
        dots.id = 10;
        dots.type = visualization_msgs::Marker::POINTS;
        dots.scale.x = dots.scale.y = 0.2;
        std_msgs::ColorRGBA color;
         color.r = 0.0; color.b = 0.0; color.g = 1.0; color.a = 1.0;
         geometry_msgs::Point topLeft;
         topLeft.x = 100;
         topLeft.y = 100;
          for(int i = 0; i<200;i++){
             for(int j=0;j<200;j++){
                 geometry_msgs::Point p;
                 p.x=i-100;
                 p.y=j-100;
                 if(obstacleBetweenPoints(p,topLeft)){
                    dots.points.push_back(p);
                    dots.colors.push_back(color);
                 }
             }
         }
         //TestsGRIDVALUE FUNCTION
        //  for(int i = 0; i<5;i++){
        //      for(int j=0;j<30;j++){
        //          geometry_msgs::Point p;
        //          p.x=i;
        //          p.y=j;
        //          if(getGridValueFromStandardXYCord(p)>=0){
        //             dots.points.push_back(p);
        //             dots.colors.push_back(color);
        //          }
        //      }
        //  }

        //DISPLAYS WALLS
        // for(int i =0;i<static_cast<int>(grid.data.size());i+=20){
        // if(grid.data.at(i)>20){
        // geometry_msgs::Point p;
        // p.x = i%width *resolution + origin.x;
        // p.y= i/height *resolution +origin.y;
        // dots.points.push_back(p);
        // dots.colors.push_back(color);
        // }
        // }
        // SHOWS OBSTACLE AVOIDANCE
        // geometry_msgs::Point zero;
        // zero.x = 0;
        // zero.y = 0;
        //  for(int i =0;i<static_cast<int>(grid.data.size());i+=20){
        //     geometry_msgs::Point p;
        //     p.x = i%width *resolution;
        //      p.y= i/height *resolution;
        // if(obstacleBetweenPoints(zero,p)){
        //    //p.x+=origin.x;
        //   // p.y+=origin.y; 
        // dots.points.push_back(p);
        // dots.colors.push_back(color);
        // }
        // }
        dots.header.stamp = ros::Time::now();
        vis_pub.publish(dots);
        dots.points.clear();
        dots.colors.clear();


    }
    geometry_msgs::Point carLocationToStandardXY(geometry_msgs::Point pointInCarFrame){
        geometry_msgs::Point newPoint;
        newPoint.x = -pointInCarFrame.y;
        newPoint.y = pointInCarFrame.x;
        return newPoint;
    }
    int8_t getGridValueFromStandardXYCord(geometry_msgs::Point pointInXYFrame){
        double newX = pointInXYFrame.x + grid.info.width * grid.info.resolution /2;
        double newY = pointInXYFrame.y + grid .info.height * grid.info.resolution/ 2;
        int index = static_cast<int>(newX/grid.info.resolution) + static_cast<int>(newY/grid.info.resolution)*grid.info.width; 
        if(index<0 || index>=static_cast<int>(grid.data.size())){
            return -1;
        }
        return grid.data.at(index);
    }
     bool obstacleBetweenPoints(geometry_msgs::Point point1, geometry_msgs::Point point2){
            //new attemp should be much faster as it will check as few cells as it needs to
            //can also be shotened even further by reducing accuracy
            //May want to increase the number of y cells check in cases of near vertical lines - accuracy is currently spotty in these cases
            if(abs(point1.x -point2.x)>grid.info.resolution){
                if(point1.x>point2.x){
                    //ensures point2 is bigger
                    double tempX = point2.x;
                    double tempY = point2.y;
                    point2.x = point1.x;
                    point2.y = point1.y;
                    point1.x = tempX;
                    point1.y = tempY;
                }
                double xDistanceToCover = point2.x - point1.x;
                double indiciesToCover = xDistanceToCover/grid.info.resolution;
                double slope = (point2.y - point1.y) / (point2.x-point1.x);
                for(int i =0;i<indiciesToCover;i++){
                    geometry_msgs::Point pointBetweenLine;
                    pointBetweenLine.x = point1.x + i*grid.info.resolution;
                    pointBetweenLine.y = point1.y + i*grid.info.resolution*slope;
                    int8_t valueAtPoint = getGridValueFromStandardXYCord(pointBetweenLine);
                    if(valueAtPoint>20){
                        return true;
                    }
                    }
                }
            return false;
    }
    
    void pose_callback(const tf2_msgs::TFMessage::ConstPtr& poses){
        geometry_msgs::TransformStamped poseOfCar;
        bool poseFound = false;
        for(geometry_msgs::TransformStamped pose: poses->transforms){
            if(pose.child_frame_id == "base_link"){
                poseOfCar = pose;
                poseFound = true;
            }
        }
        if(poseFound){
        //ROS_INFO("%s",std::to_string(poseOfCar.transform.translation.x).c_str());
        //ROS_INFO("%s",std::to_string(poseOfCar.transform.translation.y).c_str());
        //ROS_INFO("%s",std::to_string(poseOfCar.transform.translation.z).c_str());
        tf::Quaternion q(poseOfCar.transform.rotation.x,poseOfCar.transform.rotation.y,poseOfCar.transform.rotation.z,poseOfCar.transform.rotation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        angleOfCar=yaw;
        //ROS_INFO("Yaw: %s",std::to_string(yaw).c_str());
        if(mapCreated){
            RRTStarNode carNode = RRTStarNode(poseOfCar.transform.translation.x,poseOfCar.transform.translation.y);
            RRTStarPathTree tree = RRTStarPathTree(carNode);
            int count = 0;
            while(count<1000){
                int treeIndex = 0;
               bool wasNodeAdded = tree.addNode(treeIndex,*(this));
               if(wasNodeAdded){
                   tree.reWireTree(treeIndex);
                   count++;
               }
            }
            double highestCost = 0;
            double highestIndex = 0;
            for(int i =0 ;i<static_cast<int>(tree.listOfNodes.size());i++){
                //double cost = tree.listOfNodes.at(i).getTotalCost();
                double cost = distance(tree.listOfNodes.at(i).coordinates,carNode.coordinates);
                if(cost>highestCost){
                    highestIndex = i;
                    highestCost = cost;
                }
            }
            RRTStarNode nodeToGuidePath = tree.listOfNodes.at(highestIndex);
            while(distance(nodeToGuidePath.coordinates,carNode.coordinates)>2.0){
                nodeToGuidePath = *nodeToGuidePath.lastNode;
            }
            if(!nodeToGuidePath.hasLastNode){
            ackermann_msgs::AckermannDriveStamped drive_st_msg;
            ackermann_msgs::AckermannDrive drive_msg;
            drive_msg.steering_angle = 0;
            drive_msg.speed = 0;
            drive_st_msg.drive = drive_msg;
            drive_pub.publish(drive_st_msg);
            }else{
                // INSERT CORRECT TRAJECTORY HERE
                geometry_msgs::Point differenceVector;
                differenceVector.x = nodeToGuidePath.coordinates.x-poseOfCar.transform.translation.x;
                differenceVector.y = nodeToGuidePath.coordinates.y-poseOfCar.transform.translation.y;
                double lookAheadDistance = pow(differenceVector.x,2) +pow(differenceVector.y,2);
                double rotatedX = std::cos(yaw)*differenceVector.x - std::sin(yaw)*differenceVector.y; 
                double radiusOfCurvature = lookAheadDistance * lookAheadDistance / (2 * std::abs(rotatedX));
                 double steeringAngle;
                if(radiusOfCurvature <car_length){
                    steeringAngle = (-1+2*static_cast<int>(rotatedX>0))*max_steering_angle;
                }else{
                steeringAngle =  (-1+2*static_cast<int>(rotatedX>0)) * atan(1/ ( sqrt( (pow(radiusOfCurvature,2) - pow(car_length/2,2)) / pow(car_length,2) ) ) );
                }
                //Testing Area
                // dots.header.frame_id = "map";
                // dots.ns = "test dots";
                // dots.action = visualization_msgs::Marker::ADD;
                // dots.pose.orientation.w = 1.0;
                // dots.id = 10;
                // dots.type = visualization_msgs::Marker::POINTS;
                // dots.scale.x = dots.scale.y = 0.2;
                // std_msgs::ColorRGBA color;
                // color.r = 0.0; color.b = 0.0; color.g = 1.0; color.a = 1.0;
                // RRTStarNode currentNode = nodeToGuidePath;
                // bool shouldContinueLoop =true;
                // while(shouldContinueLoop){
                //     dots.points.push_back(nodeToGuidePath.coordinates);
                //     dots.colors.push_back(color);
                //     shouldContinueLoop = nodeToGuidePath.hasLastNode;
                //     if(shouldContinueLoop){
                //     nodeToGuidePath = *nodeToGuidePath.lastNode;
                //     }
                // }
                // dots.header.stamp = ros::Time::now();
                // vis_pub.publish(dots);
                // dots.points.clear();
                // dots.colors.clear();    
                //
                ackermann_msgs::AckermannDriveStamped drive_st_msg;
                ackermann_msgs::AckermannDrive drive_msg;
                drive_msg.steering_angle = steeringAngle;
                ROS_INFO("Steering Angle: %s",std::to_string(steeringAngle).c_str());
                drive_msg.speed = 0.5;
                drive_st_msg.drive = drive_msg;
                drive_pub.publish(drive_st_msg);
            }

        }
        }
    }
     void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& map){
         ROS_INFO("Map Change");
         grid.header = map->header;
         grid.info = map->info;
         grid.data=map->data;
         //adds carRadius to avoid 
         for(int i=0;i<static_cast<int>(grid.data.size());i++){
             if(grid.data.at(i)>=0){
                 grid.data.at(i)= static_cast<int8_t>(grid.data.at(i)/2);
                 int searchLeftIndex = i%grid.info.width-static_cast<int>(avoidRadius/grid.info.resolution);
                 int searchRightIndex = i%grid.info.width+static_cast<int>(avoidRadius/grid.info.resolution);
                 int searchUpIndex = i/grid.info.width +static_cast<int>(avoidRadius/grid.info.resolution);
                 int searchDownIndex = i/grid.info.width -static_cast<int>(avoidRadius/grid.info.resolution);
                 if(searchLeftIndex /grid.info.width <0){
                     searchLeftIndex = 0;
                 }
                 if(searchRightIndex /grid.info.width>=grid.info.width){
                     searchRightIndex = grid.info.width-1;
                 }
                 if(searchDownIndex <0){
                     searchDownIndex = 0;
                 }
                 if(searchUpIndex >= grid.info.height){
                     searchUpIndex = grid.info.height-1;
                 }
                 for(int j =0;j<searchUpIndex-searchDownIndex;j++){
                     for(int k=0;k<searchRightIndex-searchLeftIndex;k++){
                         if(map->data.at((searchDownIndex +j)*grid.info.width+ searchLeftIndex + k)>80){
                            grid.data.at(i) = static_cast<int8_t>(80);
                         }
                     }
                }

             }
         }
         
         mapCreated=true;
     }
}; // end of class definition

bool RRTStarPathTree::addNode(int& nodeAdded,ReactiveMethod& reactor){
    //Make sure it can only add within certain angle
    //restrict distance
    geometry_msgs::Point newPoint;
                double xRandom = unif(re);
                double yRandom = unif(re);
                newPoint.x = startNode->coordinates.x + xRandom;
                newPoint.y = startNode->coordinates.y + yRandom;
                double closestDistance = distance(newPoint,startNode->coordinates);
                int indexOfBestPoint = 0;
                     for(int i =1;i<static_cast<int>(listOfNodes.size());i++){
                         if(distance(newPoint,listOfNodes.at(i).coordinates) < closestDistance){
                             // angle checking likely breaks at first index when it isnt at the start point because it assumes start of tree will always be at a car angle that is 0 
                                double angle = getAngle(listOfNodes.at(i).lastNode->coordinates,listOfNodes.at(i).coordinates,newPoint);
                                if(abs(angle) <M_PI*0.85){
                                    continue;
                                }
                                closestDistance= listOfNodes.at(i).getTotalCost()+distance(newPoint,listOfNodes.at(i).coordinates);
                                indexOfBestPoint = i;
                            }
                         }
                     if(indexOfBestPoint == 0){
                        double angle = getAngle(listOfNodes.at(indexOfBestPoint).coordinates,listOfNodes.at(indexOfBestPoint).coordinates,newPoint);
                        if(abs(abs(reactor.angleOfCar) -angle) >M_PI*0.15){
                                return false;
                            }
                     }
                    if(distance(newPoint,listOfNodes.at(indexOfBestPoint).coordinates)>1){
                        double changeInX = newPoint.x-listOfNodes.at(indexOfBestPoint).coordinates.x;
                        double changeInY = newPoint.y-listOfNodes.at(indexOfBestPoint).coordinates.y;
                        double multiplier = 1/sqrt(pow(changeInX,2)+pow(changeInY,2));
                        newPoint.x = listOfNodes.at(indexOfBestPoint).coordinates.x + changeInX * multiplier;
                        newPoint.y = listOfNodes.at(indexOfBestPoint).coordinates.y + changeInY * multiplier;
                    }
                    if(reactor.obstacleBetweenPoints(newPoint,listOfNodes.at(indexOfBestPoint).coordinates)){
                        return false;
                    }
                    RRTStarNode testNode = listOfNodes.at(indexOfBestPoint);
                    RRTStarNode newNodeToAdd = RRTStarNode(newPoint.x,newPoint.y,listOfNodes.at(indexOfBestPoint));
                    listOfNodes.push_back(newNodeToAdd);
                    nodeAdded = static_cast<int>(listOfNodes.size())-1;
                    return true;
}
int main(int argc, char ** argv) {
    ros::init(argc, argv, "Reactive_Method");
    ReactiveMethod rw;
    ros::spin();
    return 0;
}