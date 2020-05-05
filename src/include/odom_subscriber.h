/*

Copyright (C) 2020 Roberto Zegers Rusche  - All Rights Reserved

- Unauthorized copying of this file, via any medium is strictly prohibited
- Proprietary and confidential
- Written by Roberto Zegers Rusche <rfzegers@gmail.com>, March 2020

*/

/*
This class reads the position coordinates of the robot from the /odom topic
*/

#ifndef ODOM_SUBSCRIBER_CLASS_H_
#define ODOM_SUBSCRIBER_CLASS_H_

#include <ros/ros.h>

// imports the Odometry message type from nav_msgs required to subscribe to Odometry incoming messages
#include <nav_msgs/Odometry.h>

class OdomSubscriber
{
public:
  OdomSubscriber(ros::NodeHandle* nodehandle) : nh_(*nodehandle)
  {
    initOdomSubscriber();
  };
  bool getOdomStatus() { return received_odom_; }
  void setOdomStatus(bool new_status) { received_odom_ = new_status; }
  // return a reference to the odometry coordinates
  std::pair<double, double> &getOdomCoords() { return odom_coords_; }

private:
  ros::NodeHandle nh_;
  // subscriber declared as a private object which is set up within the constructor of this class
  ros::Subscriber odom_sub_;
  // helper method to encapsulate the details of initializing a subscriber
  void initOdomSubscriber();
  // Set odometry coordinates from odometry message
  void subscriberCallback(const nav_msgs::Odometry& msg);
  // flag to keep track whether odometry has been received or not
  bool received_odom_ = false;
  // keep track of odometry coordinates
  std::pair<double, double> odom_coords_;
};

#endif