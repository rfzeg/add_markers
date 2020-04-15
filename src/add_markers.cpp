/*
  Author: Roberto Zegers
  Date: 2019-03-22
*/

// ros
#include <ros/ros.h>
// ros package to access package directory
#include <ros/package.h>
// marker visualization message
#include <visualization_msgs/Marker.h>
// pose message
#include <geometry_msgs/Pose.h>
// stamped pose message
#include <geometry_msgs/PoseStamped.h>
// tf2 matrix
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
// yaml file handling
#include <yaml-cpp/yaml.h>
// stream library to both read and write from/to files
#include <fstream>
// imports the Odometry message type from nav_msgs required to subscribe to Odometry incoming messages
#include <nav_msgs/Odometry.h>
// incorporate file that defines the class "RvizMarkersPub"
#include "include/rviz_markers_pub.h"

// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template<typename T>
void operator >> ( const YAML::Node& node, T& i )
{
  i = node.as<T>();
}

// initialize global variable for current global pose
geometry_msgs::PoseStamped global_pose;

/* Parse Rviz markers from yaml file */
bool parseMarkersFromFile(std::vector<geometry_msgs::Pose> &parsed_marker_poses) // pass array of Pose messages by reference
{
  // clear vector
  parsed_marker_poses.clear();

  // declare string for input file name including the path inside the package
  std::string markers_filename = "config/markers.yaml";
  std::string markers_path_filename;
    
  // get the package path
  std::string pkg_path= ros::package::getPath( "add_markers" ); // change accordingly
  markers_path_filename = pkg_path + "/" + markers_filename;

  try
  {
    // determine if file opens, ifs is the name of the object created
    std::ifstream ifs( markers_path_filename.c_str(), std::ifstream::in );
    if ( !ifs.good() )
    {
      ROS_FATAL( "parseMarkersFromFile() could not open file" );// if it cannot open the file check path, package name
      return false; 
    }
    YAML::Node yaml_node;
    // load a YAML string
    yaml_node = YAML::Load(ifs);
    // read data of new yaml file
    const YAML::Node &wp_node_tmp = yaml_node[ "markers" ];
    const YAML::Node *wp_node = wp_node_tmp ? &wp_node_tmp : NULL;

    if (wp_node != NULL)
    {
      // loop over all the sub-nodes
      for (int i = 0; i < wp_node->size(); i++)
      {
        // declare 'current_point' which is used to keep each marker's pose
        geometry_msgs::Pose current_point;
        double yaw;

        (*wp_node)[i]["point"]["x"] >> current_point.position.x;
        (*wp_node)[i]["point"]["y"] >> current_point.position.y;
        
        (*wp_node)[i]["point"]["th"] >> yaw;
        
        /* convert degrees to quaternion */
        // 1. convert degrees to radians // TO-DO: make sure angle is normalized
        yaw = yaw * M_PI / 180.;
        // declare quaternion
        tf2::Quaternion q;
        // 2. convert to quaternion
        q.setRPY( 0., 0., yaw );
        current_point.orientation.x = q.x();
        current_point.orientation.y = q.y();
        current_point.orientation.z = q.z();
        current_point.orientation.w = q.w();
        
        parsed_marker_poses.push_back(current_point);
      }
    }
    else
    {
      ROS_FATAL( "parseMarkersFromFile() failed, wp_node == NULL" ); 
      return false;
    }
  }
  catch (YAML::ParserException &e)
  {
    ROS_FATAL( "parseMarkersFromFile() failed, YAML::ParserException" );
    return false;
  }
  catch (YAML::RepresentationException &e)
  {
    ROS_FATAL( "parseMarkersFromFile() failed, YAML::RepresentationException" );
    return false;
  }
  return true;
}

/* Get current pose from odometry topic */
void odomListener(const nav_msgs::Odometry::ConstPtr& msg)
{
    // Populate current global pose header
    global_pose.header.stamp = ros::Time::now();
    // Populate current global pose position data
    global_pose.pose.position.x = msg->pose.pose.position.x;
    global_pose.pose.position.y = msg->pose.pose.position.y; 
    global_pose.pose.position.z = msg->pose.pose.position.z;
    // Populate current global pose orientation data
    global_pose.pose.orientation = msg->pose.pose.orientation;
}

/* Calculate euclidean distance between two points */
double pose2XYdistance(const geometry_msgs::PoseStamped& global_pose, double goal_x, double goal_y) {
    return hypot(goal_x - global_pose.pose.position.x, goal_y - global_pose.pose.position.y);
  }

/* Check if current pose is equal to waypoint position considering a tolerance */
bool isWaypointReached(const geometry_msgs::PoseStamped& global_pose, const geometry_msgs::Pose& wp_pose)
{
    double xy_goal_tolerance = 0.25; // set tolerance as required
    double goal_x = wp_pose.position.x; // goal_pose.pose.position.x;
    double goal_y = wp_pose.position.y; // goal_pose.pose.position.y;
    //  check to see if we've reached the waypoint position
    if(pose2XYdistance(global_pose, goal_x, goal_y) <= xy_goal_tolerance) {
      return true;
     }
     else 
     {
       return false;
     }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers"); // create ROS node
  ros::NodeHandle n; // start node
  ros::Subscriber sub = n.subscribe("/odom", 1000, odomListener);
  // init RvizMarkersPub object
  RvizMarkersPub rvizMarkersPub(&n);
  // declare an array of Pose messages to store the parsed data from the selected YAML file
  std::vector<geometry_msgs::Pose> parsed_marker_poses;
  // declare waypoint element that holds one specific pose
  geometry_msgs::Pose waypoint_element;
  // integer that keeps track of the shape, initial shape type is set to be 'CUBE'
  // e.g. ARROW=0, CUBE=1, SPHERE=2, CYLINDER=3, TEXT_VIEW_FACING=9, MESH_RESOURCE=10
  uint32_t shape = visualization_msgs::Marker::CUBE;

  // variable used to cycle trough different tasks
  uint8_t task = 0;
  bool built = parseMarkersFromFile(parsed_marker_poses);
  
  if ( !built )
  {
    ROS_FATAL( "Building markers from a file failed" );
    return 0;
  }

  // make best effort at maintaining a particular rate for a loop
  ros::Rate rate(0.5);// in Hz
  while(ros::ok())
  {
  switch (task)
    {
    case 0:
      ROS_INFO_ONCE("At start location");
      // funtion call to build a Rviz marker message and publish it
      rvizMarkersPub.newVisMsg(parsed_marker_poses, 0, "ADD", 0);
      task = 1;
      break;
    
    case 1:
      ROS_INFO_ONCE("Moving to pick up place");
      // check if the pick up place is reached
      waypoint_element = parsed_marker_poses[0];
      if (isWaypointReached(global_pose, waypoint_element))
      {
        task = 2;
      }
      break;
    
    case 2:
      ROS_INFO_ONCE("At pick up place");
      // Wait 5 seconds to simulate a pickup
      // construct a ros::Duration object, then call its sleep() method
      ros::Duration(5, 0).sleep();
      // funtion call to build a Rviz marker message and publish it
      rvizMarkersPub.newVisMsg(parsed_marker_poses, 0, "DELETE", 0);
      // move to next task
      task = 3;
      break;
    
    case 3:
      ROS_INFO_ONCE("Moving to drop off zone");
      // check if the drop off zone is reached
      waypoint_element = parsed_marker_poses[1];
      if (isWaypointReached(global_pose, waypoint_element))
      {
        // move to next task
        task = 4;
      }
      break;
    
    case 4:
      ROS_INFO_ONCE("At drop off zone");
      // funtion call to build a Rviz marker message and publish it
      rvizMarkersPub.newVisMsg(parsed_marker_poses, 1, "ADD", 0);
      break;
    }
    // will call all the callbacks waiting to be called at that point in time
    ros::spinOnce();
    // tracks time since last rate.sleep() and stops the right amount of time to match the rate()
    rate.sleep();
  }
  return 0;
}