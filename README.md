<h1>add_markers ROS Package <img src="docs/imgs/ros_cpp_logo.png" align="right" width="143" height="30" /> </h1>

Author: R. Zegers

## Description
ROS node that reads in Rviz marker positions from a YAML file and publishes them to Rviz as required.  
It also gets the current pose from the odometry topic, checks if robot's pose is equal to the position of one of the markers and
uses that information to decide whether or not to to publish a marker to Rviz.

## Dependencies
- yaml-cpp 05

After installing ROS, the yaml-cpp library gets installed as a rosdep system dependency. Rosdep gets installed (when following the steps in the official ROS wiki page) by the instruction below:  
`$ rosdep install --from-paths -i -y src`  
> To use yaml-cpp with a new ROS package make sure the CMakeLists.txt file contains the following instruction so the compiler can link
> against the yaml-cpp library:  
> `target_link_libraries(${PROJECT_NAME} yaml-cpp)`  
> Also the package.xml manifest has to be edited to add the following:  
> `<build_depend>common_rosdeps</build_depend>`  

## Repository architecture 
### Source code files :
+ **add_markers.cpp** : load Rviz Markers from YAML file and publish them when called in main()
 
### Directories :
+ **src/** : (required) main programm source code **add_markers.cpp** and static library source code files
+ **include/** : (required) header files that define the functionalities of the static libraries
+ **config/** : (required) YAML files that stores the markers and their attributes
+ **rviz/** : (optional) Rviz configuration files ready for displaying visual markers
+ **launch/** : (optional) launch files for this package

## YAML file structure

This node imports Rviz Marker data from YAML files which can store them in a persitent manner and human-readable format. YAML files can be editted manually to add/delete markers, change their properties or add/remove comments (denoted by the # sign). When doing so, it is important to take enough care to not break the formatting rules described below or the markers will not load properly. 
The data structure to which a yaml file must adhere in order to be compatible with this node is the following:

```yml
# The document root contains a sequence (each entry denoted by leading dash '-')
- name: "box1" # marker elements are stored as key/value pairs
  type: "cube" # values can be accesed using square bracket lookup: marker_node["type"] 
  frame_id: "map"
  position: [3.5, 0.0, 2.5] # some marker keys contain sequences (lists) as values
  orientation: [0.0, 0.0, 0.0, 1.0] # lists are accesed using a position index: marker_node["position"][0]
--- 
# three dashes indicate a YAML document inside the YAML file, this syntax element is ignored by the parser
- name: "sphere1" # all markers begin with a dash (-)
  type: "sphere"
  frame_id: "map"
  position: [5.5, 7.0, 4.0]
  orientation: [0.0, 0.0, 0.0, 1.0]
```

The root-level node is a sequence node (list) where each sequence entry (denoted by '-') holds the data structure that defines one Rviz marker.
Each marker entry has keys beneath it. Some marker keys contain nested sequences (lists) as values. Note that all marker keys must be prefixed with the same amount of spaces, in the example below two spaces are used, the number of spaces can vary from file to file, but tabs are not allowed.  

**Considerations:**
- The keys 'name', 'type', 'frame_id' hold data as strings.  
  On a side note: the key 'name' is formatted using a so-called 'compact nested map', that is to say: the first nested key-value pair starts in-line with the leading hyphen (-) for a more compact notation.  
- The allowed values for the **type** field are: "cube", "sphere", "cylinder", "line_strip", "sphere_list" and "points".  
- The value field for the keys 'position' and 'orientation' contain a nested sequence formatted inline (list members are enclosed in square brackets and separated by commas).  
- The field "points" is optional. It is required only for the markers of type "line_strip", "sphere_list" and "points" as it contains the vertices. All vertices must be formatted in a two levels deep nested map structure, like so:  
  ```yml
  - name: "line_strip1"
    type: "line_strip"
    frame_id: "map"
    position: [0.0, 0.0, 0.0]
    orientation: [0.0, 0.0, 0.0, 1.0]
    points:
    - point:
        x: -5.74814
        y: -5.68642
    - point:
        x: -16.4248
        y: -5.68229
    - point:
        x: -17.2092
        y: -5.30964
    - point:
        x: -17.6281
        y: -4.5394
  ```
All markers contained in one YAML file are parsed into one [MarkerArray message](http://docs.ros.org/melodic/api/visualization_msgs/html/msg/MarkerArray.html), regardless of whether those markers are located in different YAML documents or not (documents inside YAML files are separated using three dashes --- on their own line).  

## Direct usage:

- Clone this repository into a ROS catkin workspace
- Build and source the workspace
- To launch this package including Rviz: `$ roslaunch add_markers add_markers.launch`  

## Rviz configuration

To view the markers, run rviz:  
`$ rosrun rviz rviz`  

If required, set the Fixed Frame field to "/map" or the frame the marker was set under `marker_message_.header.frame_id`.  
Then add a Markers display. Verify that the topic specified is the same as the one being published.

## Class Interfaces 

**MarkerParser**  
The `parseMarkersFromFile()` method take in as argument a visualization_msgs::MarkerArray object, reads in the markers from a file (hard coded filename) and populates the MarkerArray it received.  

**rvizMarkersPub**  
The `newVisMsg()` method take in as argument a visualization_msgs::MarkerArray object populates with Marker messages and a string that can be "ADD",  "DELETE" or "DELETE ALL" which is passed in to the action field of all markers. Then it publishes that MarkerArray message.  

---

> ## Example usage together with a waypoint generator node in Gazebo:  
> 
> + Clone following repositories to a catkin workspace (for example ~/catkin_ws/src):
>   A Gazebo simulated environment:  
>   `git clone https://github.com/rfzeg/service_bot.git`  
>   A URDF robot model for simulation:  
>   `git clone https://github.com/rfzeg/dumpster.git`  
>   A package to give a robot a predefined set of waypoints:  
>   `git clone https://github.com/rfzeg/navi_goals.git`  
>   This package:  
>   `git clone https://github.com/rfzeg/add_markers.git`  
> 
> + If not already present, install xterm (required to run the bash script):
>   `sudo apt-get install xterm`
>   
> + Build and source your workspace :
>   ```
>   cd ~/catkin_ws  
>   catkin_make  
>   source devel/setup.bash  
>   ```
>     
> + Run the project with the following bash script provided inside the service_bot package:
>   `./home_service.sh`
> 
>   Or run each node one by one manually in separated terminal instances.

## Troubleshooting
- **Problem**: Node crashes with error message: terminate called after throwing an instance of 'YAML::ParserException'
  what():  yaml-cpp: error at line 2, column 2: end of sequence not found  
  **Reason**: Probably you have used tabs instead of spaces.  
  **Solution**: convert all tabs to spaces (leading and non-leading).  

- **Problem**: Node stops execution with error message: Client [/rviz] wants topic /visualization_marker to have datatype/md5sum [visualization_msgs/Marker/4048c9de2a16f4ae8e0538085ebf1b97], but our version has [visualization_msgs/MarkerArray/d155b9ce5188fbaf89745847fd5882d7]. Dropping connection.  
  **Reason**: Probably the message type specified when creating the publisher (`marker_pub = nh_.advertise<_msg\_type_>`) is different from the message type passed in to it (e.g. in `rvizMarkersPub.newVisMsg(parsed_markers, "ADD");`).  
  **Solution**: make sure visualization_msgs/Marker or visualization_msgs/MarkerArray are used in a consistent manner.  

## Resources:
- [Markers: Sending Basic Shapes (C++)](http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Basic%20Shapes)
- [YAML Ain’t Markup Language (YAML™) Version 1.2](https://yaml.org/spec/1.2/spec.html)
- [yaml-cpp Tutorial](https://github.com/jbeder/yaml-cpp/wiki/Tutorial)
