/*
  Author: Roberto Zegers
  Date: 2019-03-22
*/

#ifndef RVIZ_MARKERS_PUB_CLASS_H_
#define RVIZ_MARKERS_PUB_CLASS_H_

// ros
#include <ros/ros.h>
// ros package to access package directory
// #include <ros/package.h>

// message types used
// marker visualization message
#include <visualization_msgs/Marker.h>
// pose message
#include <geometry_msgs/Pose.h>
// stamped pose message
#include <geometry_msgs/PoseStamped.h>
// pose array message
#include <geometry_msgs/PoseArray.h>

class RvizMarkersPub
{
public:
    RvizMarkersPub(ros::NodeHandle* nodehandle);
    ros::Publisher marker_pub;
    void setMarkerType(uint32_t new_shape);
    visualization_msgs::Marker getMessage(){return marker_message_;}
    void buildVisMsg(std::vector<geometry_msgs::Pose> rviz_markers, int index, std::string action);
    // publish message
    void pub();

private:
    ros::NodeHandle nh_;
    // int to hold the marker type, ARROW=0, CUBE=1, SPHERE=2, CYLINDER=3, TEXT_VIEW_FACING=9, MESH_RESOURCE=10
    uint32_t shape_ = 1;
    
    // helper method to encapsulate the details of initializing publisher
    void initializePublishers();

    // declare a marker visualization message object
    visualization_msgs::Marker marker_message_;
};

#endif

