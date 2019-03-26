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

// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template<typename T>
void operator >> ( const YAML::Node& node, T& i )
{
  i = node.as<T>();
}

// initialize global variable for current global pose
geometry_msgs::PoseStamped global_pose;

/*
   Parse Rviz markers from yaml file
*/
bool parseMarkersFromFile(std::vector<geometry_msgs::Pose> &rviz_markers) // pass array of Pose messages by reference
{
  // clear waypoints vector
  rviz_markers.clear();

  // declare string for input file name including the path inside the package
  std::string markers_filename = "config/markers.yaml";
  std::string markers_path_filename;
    
  // get the package path
  std::string pkg_path= ros::package::getPath( "add_markers" ); // change accordingly
  markers_path_filename = pkg_path + "/" + markers_filename;

  try
  {
    // determine if file opens
    std::ifstream ifs( markers_path_filename.c_str(), std::ifstream::in ); //ifs is the name of the object created
    if ( !ifs.good() )
    {
      ROS_FATAL( "parseMarkersFromFile() could not open file" );// if it cannot open the file check path, package name
      return false; 
    }
    YAML::Node yaml_node;
    // load file
    yaml_node = YAML::Load(ifs);
    // read data of new yaml file
    const YAML::Node &wp_node_tmp = yaml_node[ "markers" ];
    const YAML::Node *wp_node = wp_node_tmp ? &wp_node_tmp : NULL;

    if (wp_node != NULL)
    {
      // loop over all the markers
      for (int i = 0; i < wp_node->size(); i++)
      {
        // declare 'current_point' which is used to keep each marker's pose
        geometry_msgs::Pose current_point;
        double yaw;

        (*wp_node)[i]["point"]["x"] >> current_point.position.x;
        (*wp_node)[i]["point"]["y"] >> current_point.position.y;
        
        (*wp_node)[i]["point"]["th"] >> yaw;
        
        /// convert degrees to quaternion
        // convert degrees to radians // TO-DO: make sure angle is normalized
        yaw = yaw * M_PI / 180.;
        // declare quaternion
        tf2::Quaternion q;
        // convert to quaternion
        q.setRPY( 0., 0., yaw );
        current_point.orientation.x = q.x();
        current_point.orientation.y = q.y();
        current_point.orientation.z = q.z();
        current_point.orientation.w = q.w();
        
        rviz_markers.push_back(current_point);
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

/*
    Build one marker visualization message
*/
void buildVisMsg(visualization_msgs::Marker &marker_message, std::vector<geometry_msgs::Pose> rviz_markers, uint32_t shape, int index)
{
  // Set the frame ID and optionally the timestamp. See the TF tutorials for information on these.
  marker_message.header.frame_id = "/map"; // reference frame relative to which the marker's pose is interpreted

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker_message.ns = "basic_shapes";  // define a namespace
  marker_message.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker_message.type = shape; // e.g. ARROW=0, CUBE=1, SPHERE=2, CYLINDER=3, TEXT_VIEW_FACING=9, MESH_RESOURCE=10

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  // add really means "create or modify".
  marker_message.action = visualization_msgs::Marker::ADD;

  // set the pose of the marker
  marker_message.pose.position.x = rviz_markers[index].position.x;
  marker_message.pose.position.y = rviz_markers[index].position.y;
  marker_message.pose.position.z = 0.5;
  marker_message.pose.orientation.x = rviz_markers[index].orientation.x;
  marker_message.pose.orientation.y = rviz_markers[index].orientation.y;
  marker_message.pose.orientation.z = rviz_markers[index].orientation.z;
  marker_message.pose.orientation.w = rviz_markers[index].orientation.w;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker_message.scale.x = 0.5;
  marker_message.scale.y = 0.5;
  marker_message.scale.z = 0.5;
  // Set the color -- be sure to set alpha to something non-zero!
  marker_message.color.r = 0.0f;
  marker_message.color.g = 1.0f;
  marker_message.color.b = 0.0f;
  marker_message.color.a = 1.0;
  marker_message.lifetime = ros::Duration();
}  // end buildVisMsg()

/*
   Get current pose from odometry topic
*/
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

/*
   Calculate euclidean distance between two points
*/
double getWaypointPositionDistance(const geometry_msgs::PoseStamped& global_pose, double goal_x, double goal_y) {
    return hypot(goal_x - global_pose.pose.position.x, goal_y - global_pose.pose.position.y);
  }

/*
   Check if current pose is equal to waypoint position considering a tolerance
*/
bool isWaypointReached(const geometry_msgs::PoseStamped& global_pose, const geometry_msgs::Pose& wp_pose)
{
    double xy_goal_tolerance = 0.25; // set tolerance as required
    double goal_x = wp_pose.position.x; // goal_pose.pose.position.x;
    double goal_y = wp_pose.position.y; // goal_pose.pose.position.y;
    //  check to see if we've reached the waypoint position
    if(getWaypointPositionDistance(global_pose, goal_x, goal_y) <= xy_goal_tolerance) {
      return true;
     }
     else 
     {
       return false;
     }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers"); // create node
  ros::NodeHandle n; // start node
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  // Subscriber reads messages from the "odom" topic, which are of the type Odometry, and calls the 'chatterCallback' functions when it receives a message
  ros::Subscriber sub = n.subscribe("/odom", 1000, odomListener);
  // declare a marker visualization message object
  visualization_msgs::Marker marker_message;
  // declare an array of Pose messages to keep track of the pose of the markers
  std::vector<geometry_msgs::Pose> rviz_markers; 
  // integer that keeps track of the shape, initial shape type is set to be 'CUBE'
  // e.g. ARROW=0, CUBE=1, SPHERE=2, CYLINDER=3, TEXT_VIEW_FACING=9, MESH_RESOURCE=10
  uint32_t shape = visualization_msgs::Marker::CUBE;

  bool built = parseMarkersFromFile(rviz_markers); 
  
  if ( !built )
  {
    ROS_FATAL( "Building markers from a file failed" );
    return 0;
  }
  // check the number of subscribers that are currently connected to this Publisher
  while (marker_pub.getNumSubscribers() < 1)
      {
        ROS_WARN_ONCE("Please create a subscriber to the marker (e.g. add in Rviz)");
        sleep(1);
      } // end while

  ros::Rate rate(2); // in Hz, makes a best effort at maintaining a particular rate for a loop, here 2 Hz
  while (ros::ok())
  {
    //check if waypoint is reached, traversing the array of markers
    for ( int j = 0; j < rviz_markers.size(); j++ )
    {
      geometry_msgs::Pose wp_element = rviz_markers[j];
      // funtion call to check if Waypoint is reached   
      bool reached = isWaypointReached(global_pose, wp_element); 
      if ( !reached )
      {
        ROS_INFO("Waypoint [%d] not reached", j+1);
      }
      else
      {
        ROS_INFO("Waypoint [%d] reached !!!", j+1); 
      }
    }
    ros::spinOnce(); // will call all the callbacks waiting to be called at that point in time
    rate.sleep(); // keeps track of how much time since last rate.sleep() was executed and sleep for the correct amount of time to hit the rate() mark
  }
}