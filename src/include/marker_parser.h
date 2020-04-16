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

class MarkerParser {
  public:
    MarkerParser(){};
    bool parseMarkersFromFile(std::vector<geometry_msgs::Pose> &parsed_marker_poses);
    
  private:
    // loadDataFromYaml(const std::string& filename);

    // Parse Rviz markers from yaml file
    // void parseMarkersFromFile(const std::string& filename, YAML::Node& node);  
};

// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template<typename T>
void operator >> ( const YAML::Node& node, T& i )
{
  i = node.as<T>();
}

#endif