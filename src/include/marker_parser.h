/*
  Author: R. Zegers R.
  Date: 2020-04-16
*/

#ifndef MARKER_PARSER_CLASS_H_
#define MARKER_PARSER_CLASS_H_

// yaml file handling
#include <yaml-cpp/yaml.h>
// stream library to both read and write from/to files
#include <fstream>

// ros (ROS_INFO)
#include <ros/ros.h>
// ros package to access package directory
#include <ros/package.h>
// tf2 matrix
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
// pose message
#include <geometry_msgs/Pose.h>

// Rviz marker visualization messages
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class MarkerParser {
  public:
    MarkerParser(){};
    bool parseMarkersFromFile(visualization_msgs::MarkerArray &parsed_markers);
    
  private:
    void hex2ColorRGBAmsg(int index, std_msgs::ColorRGBA &color);
    // source: https://medialab.github.io/iwanthue/ Color space: all colors, Palette: 50 colors, soft(K-Means)
    std::vector<std::string> color_vector_1 = {
      "C8DBC0", "6122B2", "78DD42", "AA45E6", "D2E043",
      "5449DE", "6CD36F", "DF4BDD", "809D35", "3C238B",
      "D3B03C", "986AE5", "CADC88", "A137AF", "67D6AA",
      "D3379A", "468945", "DB6FCE", "2F471F"  "4C5FCC",
      "E18734", "5C83D5", "DE4330", "6CD2D5", "D9396C",
      "3E6F5D", "78378C", "86712C", "3B2062", "D8AB7D",
      "341B3B", "B7C5E1", "8C2D26", "5DA4D5", "A35728",
      "A57BCF", "8E9571", "93457C", "5F9098", "DD7566",
      "2A3E4C", "DCA1DC", "401C1E", "D7ABB1", "4A5182",
      "5E452B", "9187B1", "7C2746", "D2718F", "895E64", };

    // source: https://stackoverflow.com/questions/309149/generate-distinctly-different-rgb-colors-in-graphs
    std::vector<std::string> color_vector_2 = {
        "FF0000", "00FF00", "0000FF", "FFFF00", "FF00FF", "00FFFF", "FFFFFF",
        "800000", "008000", "000080", "808000", "800080", "008080", "808080",
        "C00000", "00C000", "0000C0", "C0C000", "C000C0", "00C0C0", "C0C0C0",
        "400000", "004000", "000040", "404000", "400040", "004040", "404040",
        "200000", "002000", "000020", "202000", "200020", "002020", "202020",
        "600000", "006000", "000060", "606000", "600060", "006060", "606060",
        "A00000", "00A000", "0000A0", "A0A000", "A000A0", "00A0A0", "A0A0A0",
        "E00000", "00E000", "0000E0", "E0E000", "E000E0", "00E0E0", "E0E0E0",
    };
};

// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template<typename T>
void operator >> ( const YAML::Node& node, T& i )
{
  i = node.as<T>();
}

// extraction operator, maps YAML Node data fields to Rviz Marker data fields
void operator >> (const YAML::Node &node, visualization_msgs::Marker &marker);

#endif