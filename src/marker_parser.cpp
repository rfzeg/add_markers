// incorporate file that defines the class "MarkerParser"
#include "include/marker_parser.h"

// Parse Rviz markers from yaml file, feed with a vector of Pose messages to hold the result
bool MarkerParser::parseMarkersFromFile(std::vector<geometry_msgs::Pose> &parsed_marker_poses)
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
    // determine if file opens (ifs is the name of ifstream object constructed)
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

