/*
  Author: Roberto Zegers
  Date: 2019-03-22
*/

#ifndef RVIZ_MARKERS_PUB_CLASS_H_
#define RVIZ_MARKERS_PUB_CLASS_H_

// ros
#include <ros/ros.h>

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

    void setMarkerType(uint32_t new_shape);
    // build and publish visualization marker
    void newVisMsg(std::vector<geometry_msgs::Pose> parsed_poses, int index, std::string action, int counter);
    void newVisLine(std::vector<geometry_msgs::Pose> &rviz_markers, int counter);

private:
    ros::NodeHandle nh_;
    // int to hold the marker type, ARROW=0, CUBE=1, SPHERE=2, CYLINDER=3, TEXT_VIEW_FACING=9, MESH_RESOURCE=10
    uint32_t shape_ = 1;

    ros::Publisher marker_pub;

    // helper method to encapsulate the details of initializing publisher
    void initializePublishers();
};

#endif

