/*
  Author: Roberto Zegers
  Date: 2019-03-22
*/

// ros
#include <ros/ros.h>

// marker visualization message
#include <visualization_msgs/Marker.h>
// pose message
#include <geometry_msgs/Pose.h>
// stamped pose message
#include <geometry_msgs/PoseStamped.h>

// imports the Odometry message type from nav_msgs required to subscribe to Odometry incoming messages
#include <nav_msgs/Odometry.h>
// incorporate file that defines the class "RvizMarkersPub"
#include "include/rviz_markers_pub.h"
// incorporate file that defines the class "MarkerParser"
#include "include/marker_parser.h"

// initialize global variable for current global pose
geometry_msgs::PoseStamped global_pose;

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

  // create a MarkerArray messages to all markers parsed from one YAML document
  visualization_msgs::MarkerArray parsed_markers;

  // declare waypoint element that holds one specific pose
  geometry_msgs::Pose waypoint_element;

  // variable used to cycle trough different tasks
  uint8_t task = 0;

  // init parser object
  MarkerParser markerParser;

  bool built = markerParser.parseMarkersFromFile(parsed_markers);

  if (!built)
  {
    ROS_FATAL("Building markers from a file failed");
    return 0;
  }
  else
  {
    ROS_INFO("YAML file has been successfully parsed");
  }

  // make best effort at maintaining a particular rate for a loop
  ros::Rate rate(0.5);// in Hz
  while(ros::ok())
  {
  switch (task)
    {
    case 0:
      ROS_INFO_ONCE("At start location");
      // build a Rviz marker message and publish it
      rvizMarkersPub.newVisMsg(parsed_markers, "ADD");
      task = 1;
      break;
    
    case 1:
      ROS_INFO_ONCE("Moving to pick up place");
      waypoint_element.position.x = parsed_markers.markers[0].pose.position.x;
      waypoint_element.position.y = parsed_markers.markers[0].pose.position.y;
      // check if the pick up place (waypoint element) is reached
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
      // build a Rviz marker message and publish it
      rvizMarkersPub.newVisMsg(parsed_markers, "DELETE");
      // move to next task
      task = 3;
      break;
    
    case 3:
      ROS_INFO_ONCE("Moving to drop off zone");
      // check if the drop off zone is reached
      waypoint_element.position.x = parsed_markers.markers[1].pose.position.x;
      waypoint_element.position.y = parsed_markers.markers[1].pose.position.y;
      if (isWaypointReached(global_pose, waypoint_element))
      {
        // move to next task
        task = 4;
      }
      break;
    
    case 4:
      ROS_INFO_ONCE("At drop off zone");
      // build a Rviz marker message and publish it
      rvizMarkersPub.newVisMsg(parsed_markers, "ADD");
      break;
    }
    // call all the callbacks waiting to be called at this point in time
    ros::spinOnce();
    // track time since last rate.sleep() and stop the right amount of time to match the rate()
    rate.sleep();
  }
  return 0;
}