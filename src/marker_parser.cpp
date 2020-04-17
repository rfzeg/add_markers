// incorporate file that defines the class "MarkerParser"
#include "include/marker_parser.h"

// Parse Rviz markers from yaml file, feed with a vector of Pose messages to hold the result
bool MarkerParser::parseMarkersFromFile(std::vector<visualization_msgs::Marker> &parsed_markers)
{
  // clear vector
  parsed_markers.clear();

  // declare string for input file name including the path inside the package
  std::string markers_filename = "config/markers.yaml";
  std::string markers_path_filename;

  // get the package path, change hard coded string accordingly
  std::string pkg_path = ros::package::getPath("add_markers");
  markers_path_filename = pkg_path + "/" + markers_filename;

  std::vector<YAML::Node> yaml_docs;
  try
  {
    // Load the YAML file into vector that contains all yaml documents
    yaml_docs = YAML::LoadAllFromFile(markers_path_filename);
  }
  catch (YAML::BadFile &ex)
  {
    ROS_ERROR("%s", ex.what());
    ROS_ERROR("Failed to load file %s", markers_path_filename.c_str());
    return false;
  }

  int marker_counter = 0;

  // Iterate through all documents in the file
  for (const auto &yaml_doc : yaml_docs)
  {
    ROS_DEBUG("New YAML document in YAML file!");
    // read data of new yaml document, yaml_doc["marker"] evaluates to false when the key its nonexistent
    // yaml_doc["marker"] pulls out a reference to the "marker" key in yaml_doc, then it assigns it to &wp_node_tmp
    // internally yaml-cpp iterates through the keys until it find a key that equals "marker"
    const YAML::Node &wp_node_tmp = yaml_doc[ "marker" ];
    // find the correct node, if it is not found, it will be left as a null node 
    const YAML::Node *wp_node = wp_node_tmp ? &wp_node_tmp : NULL;

    if (wp_node != NULL)
    {
      // loop over the children of root node ("markers")
      for (int i = 0; i < wp_node->size(); i++)
      {
        // declare variable used to keep each marker
        visualization_msgs::Marker current_marker;
        // id field needs to be distinct from existing markers in order to enable multiple markers to exist at the same time
        current_marker.id = marker_counter;
        double yaw;

        // set the pose of the marker, accessing the node's keys directly
        (*wp_node)[i]["point"]["x"] >> current_marker.pose.position.x;
        (*wp_node)[i]["point"]["y"] >> current_marker.pose.position.y;
        (*wp_node)[i]["point"]["th"] >> yaw;

        /* convert degrees to quaternion */
        // 1. convert degrees to radians
        // TO-DO: make sure angle is normalized
        yaw = yaw * M_PI / 180.;
        // declare quaternion
        tf2::Quaternion q;
        // 2. convert to quaternion
        q.setRPY(0., 0., yaw);
        current_marker.pose.orientation.x = q.x();
        current_marker.pose.orientation.y = q.y();
        current_marker.pose.orientation.z = q.z();
        current_marker.pose.orientation.w = q.w();

        parsed_markers.push_back(current_marker);
      }
    }
    else
    {
      ROS_ERROR("Couldn't find any YAML::NodeType of type Map, Sequence, Scalar or Undefined, marker_node == NULL");
      return false;
    }
  marker_counter++;
  }
  return true;
}
