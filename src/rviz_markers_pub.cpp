// incorporate file that defines the class "RvizMarkersPub"
#include "include/rviz_markers_pub.h"

RvizMarkersPub::RvizMarkersPub(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ // constructor
    initializePublishers();
}

// member helper function to set up publishers
void RvizMarkersPub::initializePublishers()
{
  ROS_INFO("Initializing Publishers");
  marker_pub = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1);

  // test/wait to make sure a subscriber is currently connected to this Publisher
  // because it requires time to create a connection between nodes, the connection
  // is confirmed in order to avoid having to enter a loop to publish a marker
  // or risk that te message is sent but not received
  while (marker_pub.getNumSubscribers() < 1)
  {
    ROS_WARN_ONCE("Waiting until Rviz Marker publisher (visualization_msgs::MarkerArray) and subscriber have established a connection (e.g. add display in Rviz)");
    sleep(1);
  }
}

/* Builds one marker visualization message and publishes it */
void RvizMarkersPub::newVisMsg(visualization_msgs::MarkerArray &parsed_marker, std::string action)
{
for(int i = 0; i < (int)parsed_marker.markers.size(); ++i)
  {
  
  // set action field to specify what to do with the marker, options are ADD, DELETE, and DELETEALL
  if((action.compare("ADD")) == 0) {
      parsed_marker.markers[i].action = visualization_msgs::Marker::ADD;
  }
  else if ((action.compare("DELETE")) == 0) {
      parsed_marker.markers[i].action = visualization_msgs::Marker::DELETE;
  }
  else if ((action.compare("DELETEALL")) == 0) {
      parsed_marker.markers[i].action = visualization_msgs::Marker::DELETEALL;
  }
  else {
      ROS_ERROR("No (or wrong) action field set to specify how the marker behaves (options are ADD, DELETE, DELETEALL)");
  }
  }

  marker_pub.publish(parsed_marker);
}

/* Builds visualization message containing one line and publishes it */
void RvizMarkersPub::newVisLine(std::vector<geometry_msgs::Pose> &rviz_markers, int counter)
{
  visualization_msgs::Marker m;
  m.header.frame_id = "/map";
  m.header.stamp = ros::Time::now();
  m.id = counter;
  m.action = visualization_msgs::Marker::ADD;
  m.scale.x = 0.1;
  m.type = visualization_msgs::Marker::LINE_STRIP; // set marker type
  m.color.r = 1; m.color.g = 0; m.color.b = 0; m.color.a = 1;
  // specify how long this marker should stick around before being automatically deleted.
  m.lifetime = ros::Duration();

  for (auto it : rviz_markers) {
    geometry_msgs::Point ps;
    ps.x = it.position.x;
    ps.y = it.position.y;
    ps.z = it.position.z;
    m.points.push_back(ps);
  }

  marker_pub.publish(m);
}
