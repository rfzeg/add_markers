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
// stamped point message
#include <geometry_msgs/PointStamped.h>
// tf2 matrix
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
// yaml file handling
#include <yaml-cpp/yaml.h>
// stream library to both read and write from/to files
#include <fstream>

// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template<typename T>
void operator >> ( const YAML::Node& node, T& i )
{
  i = node.as<T>();
}

/*
   Build Rviz_markers from yaml file
*/
bool buildMarkersFromFile(std::vector<geometry_msgs::PointStamped> &rviz_markers) // pass array of PointStamped messages by reference
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
      ROS_FATAL( "buildMarkersFromFile() could not open file" );// if it cannot open the file check path, package name
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
        // declare 'point' which is used to keep each marker's parameters/coordinates (double)
        geometry_msgs::PointStamped point;

        (*wp_node)[i]["point"]["x"] >> point.point.x;
        (*wp_node)[i]["point"]["y"] >> point.point.y;
        (*wp_node)[i]["point"]["th"] >> point.point.z; // 'th' from here on is 'z'
        rviz_markers.push_back(point);  // 
      }
    }
    else
    {
      ROS_FATAL( "buildMarkersFromFile() failed, wp_node == NULL" ); 
      return false;
    }
  }
  catch (YAML::ParserException &e)
  {
    ROS_FATAL( "buildMarkersFromFile() failed, YAML::ParserException" );
    return false;
  }
  catch (YAML::RepresentationException &e)
  {
    ROS_FATAL( "buildMarkersFromFile() failed, YAML::RepresentationException" );
    return false;
  }
  return true;
}


/*
   Run markers
*/
void run(visualization_msgs::Marker &marker_drop, std::vector<geometry_msgs::PointStamped> rviz_markers, uint32_t shape, int index)
{
  // Set the frame ID and optionally the timestamp. See the TF tutorials for information on these.
  marker_drop.header.frame_id = "/map"; // reference frame relative to which the marker's pose is interpreted

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker_drop.ns = "basic_shapes";  // namespace
  marker_drop.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker_drop.type = shape; // e.g. ARROW=0, CUBE=1, SPHERE=2, CYLINDER=3, TEXT_VIEW_FACING=9, MESH_RESOURCE=10

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  // add really means "create or modify".
  marker_drop.action = visualization_msgs::Marker::ADD;

  // set the pose of the marker
  marker_drop.pose.position.x = rviz_markers[index].point.x;
  marker_drop.pose.position.y = rviz_markers[index].point.y;
  marker_drop.pose.position.z = 0.5;

  // convert the degrees to quaternion
  double yaw = rviz_markers[index].point.z * M_PI / 180.;
  tf2::Quaternion q;
  q.setRPY( 0., 0., yaw );
  marker_drop.pose.orientation.x = q.x();
  marker_drop.pose.orientation.y = q.y();
  marker_drop.pose.orientation.z = q.z();
  marker_drop.pose.orientation.w = q.w();

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker_drop.scale.x = 0.5;
  marker_drop.scale.y = 0.5;
  marker_drop.scale.z = 0.5;
  // Set the color -- be sure to set alpha to something non-zero!
  marker_drop.color.r = 0.0f;
  marker_drop.color.g = 1.0f;
  marker_drop.color.b = 0.0f;
  marker_drop.color.a = 1.0;
  marker_drop.lifetime = ros::Duration();
}  // end run()


int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers"); // create node
  ros::NodeHandle n; // start node
  ros::Rate r(0.2); // makes a best effort at maintaining a particular rate for a loop, here 0.2 Hz
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  visualization_msgs::Marker marker_drop; // declare a visualization marker

  // declare an array of PointStamped messages to keep track of the pose of the markers
  std::vector<geometry_msgs::PointStamped> rviz_markers; 

  // integer that keeps track of the shape, initial shape type is set to be 'CUBE'
  // e.g. ARROW=0, CUBE=1, SPHERE=2, CYLINDER=3, TEXT_VIEW_FACING=9, MESH_RESOURCE=10
  uint32_t shape = visualization_msgs::Marker::CUBE;

  bool built = buildMarkersFromFile(rviz_markers); 
  if ( !built )
  {
    ROS_FATAL( "Building markers from a file failed" );
    return 0;
  }

  while (ros::ok())
  {
    // publishes Markers
    for ( int j = 0; j < rviz_markers.size(); j++ )
    {
      // funtion call to display Rviz markers, traversing the array of markers
      run(marker_drop, rviz_markers, shape, j);

      // Publish the marker
      while (marker_pub.getNumSubscribers() < 1)
      {
        ROS_WARN_ONCE("Please create a subscriber to the marker (e.g. add marker in Rviz)");
        sleep(1);
      } // end while
      
      marker_pub.publish(marker_drop);
      r.sleep(); // keeps track of how much time since last r.sleep() was executed and sleep for the correct amount of time to hit the ros::Rate mark
    }
  }
}