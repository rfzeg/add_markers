/*

Copyright (C) 2020 Roberto Zegers Rusche  - All Rights Reserved

- Unauthorized copying of this file, via any medium is strictly prohibited
- Proprietary and confidential
- Written by Roberto Zegers Rusche <rfzegers@gmail.com>, March 2020

*/

/*
This class reads the position coordinates of the robot from the /odom topic
*/

#include "include/odom_subscriber.h"

// member helper function to set up subscriber
void OdomSubscriber::initOdomSubscriber()
{
  ROS_DEBUG("Initializing odometry subscriber");
  // Subscribe to published point
  odom_sub_ = nh_.subscribe("/odom", 1, &OdomSubscriber::subscriberCallback,this); 
}

void OdomSubscriber::subscriberCallback(const nav_msgs::Odometry& msg) {
  odom_coords_.first = msg.pose.pose.position.x;
  odom_coords_.second = msg.pose.pose.position.y;
  ROS_DEBUG("I heard odometry coordinates: (%f,%f)", odom_coords_.first, odom_coords_.second);
  received_odom_ = true;
}
