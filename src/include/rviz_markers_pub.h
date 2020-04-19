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
#include <visualization_msgs/MarkerArray.h>


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

    // build and publish visualization marker
    void newVisMsg(visualization_msgs::MarkerArray &parsed_marker, std::string action);
    void newVisLine(std::vector<geometry_msgs::Pose> &rviz_markers, int counter);

private:
    ros::NodeHandle nh_;

    ros::Publisher marker_pub;

    // helper method to encapsulate the details of initializing publisher
    void initializePublishers();
};

#endif

