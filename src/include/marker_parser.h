/*
  Author: R. Zegers R.
  Date: 2020-04-16
*/

#ifndef MARKER_PARSER_CLASS_H_
#define MARKER_PARSER_CLASS_H_

// yaml file handling
#include <yaml-cpp/yaml.h>
// stream library to both read and write from/to files
#include <fstream>

// ros (ROS_INFO)
#include <ros/ros.h>
// ros package to access package directory
#include <ros/package.h>
// tf2 matrix
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
// pose message
#include <geometry_msgs/Pose.h>

// Rviz marker visualization messages
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class MarkerParser {
  public:
    MarkerParser(){};
    bool parseMarkersFromFile(visualization_msgs::MarkerArray &parsed_markers);
    
  private:
};

// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template<typename T>
void operator >> ( const YAML::Node& node, T& i )
{
  i = node.as<T>();
}

// extraction operator, maps YAML Node data fields to Rviz Marker data fields
void operator >> (const YAML::Node &node, visualization_msgs::Marker &marker);

#endif