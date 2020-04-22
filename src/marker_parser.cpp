// incorporate file that defines the class "MarkerParser"
#include "include/marker_parser.h"

// extraction operator, maps YAML Node data fields to Rviz Marker data fields
void operator >> (const YAML::Node &node, visualization_msgs::Marker &marker)
{
  std::string type = node["type"].as<std::string>();
  // types enumerated in visualization_msgs/Marker message:
  // http://docs.ros.org/melodic/api/visualization_msgs/html/msg/Marker.html
  if (type == "cube")
  {
    marker.type = visualization_msgs::Marker::CUBE;
  }
  else if (type == "sphere")
  {
    marker.type = visualization_msgs::Marker::SPHERE;
  }
  else if (type == "cylinder")
  {
    marker.type = visualization_msgs::Marker::CYLINDER;
  }
  else if (type == "line_strip")
  {
    marker.type = visualization_msgs::Marker::LINE_STRIP;
  }
  else if (type == "sphere_list")
  {
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
  }
  else if (type == "points")
  {
    marker.type = visualization_msgs::Marker::POINTS;
  }
  else
  {
    ROS_WARN("Wrong value for 'type' field in YAML file, using default 'cube'");
    ROS_INFO("Only \"cube\", \"sphere\", \"cylinder\", \"line_strip\" \"sphere_list\" and \"points\"  (case sensitive) are supported");
    marker.type = visualization_msgs::Marker::CUBE;
  }

  marker.header.frame_id = node["frame_id"].as<std::string>();

  // retrieve a sub-node by quering the root 'map node' using a key value
  const YAML::Node& pos = node["position"];
  // get the values from the 'sequence node' pos by using a position index that start from 0
  marker.pose.position.x = pos[0].as<float>();
  marker.pose.position.y = pos[1].as<float>();
  marker.pose.position.z = pos[2].as<float>();

  const YAML::Node& quat = node["orientation"];
  marker.pose.orientation.x = quat[0].as<float>();
  marker.pose.orientation.y = quat[1].as<float>();
  marker.pose.orientation.z = quat[2].as<float>();
  marker.pose.orientation.w = quat[3].as<float>();

  if (YAML::Node points = node["points"])
  {
    ROS_DEBUG("Processing XY Points");
    // iterate through all point elements
    int index = 0;
    for (YAML::const_iterator it = points.begin(); it != points.end(); ++it)
    {
      // process points
      // get reference to the "it" points and create new YAML::Node reference out of it
      const YAML::Node &point = *it;
      // declare 'p' which is used to keep each coordinate pair
      geometry_msgs::Point p;
      p.x = point["point"]["x"].as<float>();
      p.y = point["point"]["y"].as<float>();
      ROS_DEBUG("Point %d (x,y): %f,%f", index, p.x, p.y);
      marker.points.push_back(p);
      // use same color as defined for the main geometry (type)
      marker.colors.push_back(marker.color);
    }
  }
}

// converts color from hex string to RGBA floats
void MarkerParser::hex2ColorRGBAmsg(int index, std_msgs::ColorRGBA &color)
{
  // convert hex string (const char[7]) to integer
  unsigned int hexValue;
  std::stringstream ss;
  ss << std::hex << color_vector_1[index % color_vector_1.size()];
  ss >> hexValue;
  // Convert from HEX color (const char[7]) to RGB floats
  color.r = ((hexValue >> 16) & 0xFF) / 255.0;
  color.g = ((hexValue >> 8) & 0xFF) / 255.0;
  color.b = ((hexValue) & 0xFF) / 255.0;
  color.a = 1.0;
}

// Parse Rviz markers from yaml file, feed with a vector of Pose messages to hold the result
bool MarkerParser::parseMarkersFromFile(visualization_msgs::MarkerArray &parsed_markers)
{
  // clear vector of markers
  parsed_markers.markers.clear();

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

  int total_counter = 0;
  // sequence iterator to iterate through all documents in the file
  for (const auto &yaml_doc : yaml_docs)
  {
    ROS_DEBUG("New YAML document in YAML file!");

    // map iterator to iterate through all rviz markers in document
    for (YAML::const_iterator it = yaml_doc.begin(); it != yaml_doc.end(); ++it)
    {
      // get reference to the "it" yaml_doc and create new YAML::Node reference out of it
      const YAML::Node &node = *it;
      // declare variable used to keep each marker
      visualization_msgs::Marker current_marker;

      // color each marker with a distinctly different colors
      std_msgs::ColorRGBA c;
      hex2ColorRGBAmsg(total_counter, c);
      current_marker.color = c;

      // map all YAML Node data fields to Rviz Marker data fields using a custom extraction operator
      node >> current_marker;

      current_marker.header.stamp = ros::Time::now();
      // id field needs to be distinct from existing markers in order to enable multiple markers to exist at the same time
      current_marker.id = total_counter;

      current_marker.scale.x = 0.5;
      current_marker.scale.y = 0.5;
      current_marker.scale.z = 0.5;

      current_marker.lifetime = ros::Duration();
      current_marker.action = visualization_msgs::Marker::ADD;
      // add current marker to vector of parsed markers
      parsed_markers.markers.push_back(current_marker);
      total_counter++;
    }
  }
  return true;
}
