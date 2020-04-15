// incorporate file that defines the class "RvizMarkersPub"
#include "include/rviz_markers_pub.h"

RvizMarkersPub::RvizMarkersPub(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ // constructor
    initializePublishers();
    // test/wait to make sure a subscriber is currently connected to this Publisher
    while (marker_pub.getNumSubscribers() < 1)
      {
        ROS_WARN_ONCE("Please create a subscriber to the marker (e.g. add in Rviz)");
        sleep(1);
      }
}

// member helper function to set up publishers
void RvizMarkersPub::initializePublishers()
{
    ROS_INFO("Initializing Publishers");
    marker_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
}

void RvizMarkersPub::setMarkerType(uint32_t new_shape){
  // set new integer that determines the marker shape
  // ARROW=0, CUBE=1, SPHERE=2, CYLINDER=3, TEXT_VIEW_FACING=9, MESH_RESOURCE=10
  shape_ = new_shape;
}

/* Build one marker visualization message */
void RvizMarkersPub::buildVisMsg(std::vector<geometry_msgs::Pose> rviz_markers, int index, std::string action)
{
  // Set the frame ID and optionally the timestamp. See the TF tutorials for information on these.
  marker_message_.header.frame_id = "/map"; // reference frame relative to which the marker's pose is interpreted

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker_message_.ns = "basic_shapes";  // define a namespace
  marker_message_.id = 0;

  // Set the marker type
  marker_message_.type = shape_;

  // set action field to specify what to do with the marker, options are ADD, DELETE, and DELETEALL
  if((action.compare("ADD")) == 0) {
      marker_message_.action = visualization_msgs::Marker::ADD;
  }
  else if ((action.compare("DELETE")) == 0) {
      marker_message_.action = visualization_msgs::Marker::DELETE;
  }
  else if ((action.compare("DELETEALL")) == 0) {
      marker_message_.action = visualization_msgs::Marker::DELETEALL;
  }
  else {
      ROS_ERROR("No (or wrong) action field set to specify how the marker behaves (options are ADD, DELETE, DELETEALL)");
  }

  // set the pose of the marker
  marker_message_.pose.position.x = rviz_markers[index].position.x;
  marker_message_.pose.position.y = rviz_markers[index].position.y;
  marker_message_.pose.position.z = 0.5;
  marker_message_.pose.orientation.x = rviz_markers[index].orientation.x;
  marker_message_.pose.orientation.y = rviz_markers[index].orientation.y;
  marker_message_.pose.orientation.z = rviz_markers[index].orientation.z;
  marker_message_.pose.orientation.w = rviz_markers[index].orientation.w;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker_message_.scale.x = 0.5;
  marker_message_.scale.y = 0.5;
  marker_message_.scale.z = 0.5;
  // Set the color -- be sure to set alpha to something non-zero!
  marker_message_.color.r = 0.0f;
  marker_message_.color.g = 1.0f;
  marker_message_.color.b = 0.0f;
  marker_message_.color.a = 1.0;
  marker_message_.lifetime = ros::Duration();
}  // end buildVisMsg()

// publish marker
void RvizMarkersPub::pub(){
  marker_pub.publish(marker_message_);
}