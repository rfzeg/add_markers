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
+ **include/** : (required) header files that defines the functionality of static libraries
+ **config/** : (required) YAML files that stores the position and orientation of markers
+ **rviz/** : (optional) Rviz configuration files ready for displaying visual markers
+ **launch/** : (optional) launch files for this package

## YAML file structure

The data structure to which a yaml file should adhere in order to be compatible with this node is the following:

```yml
# The document root contains a sequence (each entry denoted by leading dash '-')
- name: "box1" # marker elements are stored as key/value pairs,
  type: "cube" # whos values can be accesed using square bracket lookup marker_node["key"] 
  frame_id: "map"
  position: [3.5, 0.0, 2.5] # some marker elements contain sequences (lists) as values.
  orientation: [0.0, 0.0, 0.0, 1.0] # list elements can be accesed by position index eg.: marker_node["position"][0]
--- # three dashes allow to include several YAML documents into one YAML file
- name: "sphere1" # all markers begin with a dash (-)
  type: "sphere"
  frame_id: "map"
  position: [5.5, 7.0, 4.0]
  orientation: [0.0, 0.0, 0.0, 1.0]
```

Each YAML document root node is a sequence (list) where each hyphen starts a new Rviz marker.  

Each marker's data field is stored as key-value pair(*). Values can be strings (such as name, type, frame_id) or a nested sequence (such as in 'position' and 'orientation') formatted using the inline style (list members are enclosed in square brackets and separated by commas).   
_(*) More precisely as a compact nested map because the first nested key-value pair starts in-line with the leading hyphen (-) for a more compact notation._
  
All markers (each starting with a dash '-') inside a YAML file are parsed into one MarkerArray message. Even if those markers are located in different YAML documents inside the file (the separation of documents in YAML is denoted by three dashes (---) on their own line).

Important: All marker keys must be prefixed with the same amount of spaces, in the example above two spaces are used, the number of spaces can vary from file to file, but tabs are not allowed. To add comments to the YAML file use the # sign.   
  
Edit the YAML file manually to add/delete markers, change the sequence or add/remove comments. It is important to do not break the formatting rules described above or the markers will not load properly.  

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
The `setMarkerType()` method in the 'RvizMarkersPub' class populates a new Visualization Message with data and sends that message to other ROS nodes. 
Aditionally the `setMarkerType()` provides a mean to set the kind of marker being published (arrow, sphere, cube, etc.. ). The available types are enumerated in the [visualization_msgs/Marker](http://docs.ros.org/melodic/api/visualization_msgs/html/msg/Marker.html) message.  

The `parseMarkersFromFile()` in the 'MarkerParser' class reads in the markers from a file and populates the vector of Pose messages it received as argument.

---

## Example in combination with a waypoint generator node in Gazebo:  

+ Clone following repositories to a catkin workspace (for example ~/catkin_ws/src):

  A Gazebo simulated environment:  
  `git clone https://github.com/rfzeg/service_bot.git`  
  A URDF robot model for simulation:  
  `git clone https://github.com/rfzeg/dumpster.git`  
  A package to give a robot a predefined set of waypoints:  
  `git clone https://github.com/rfzeg/navi_goals.git`  
  This package:  
  `git clone https://github.com/rfzeg/add_markers.git`  

+ If not already present, install xterm (required to run the bash script):

  `sudo apt-get install xterm`
  
+ Build and source your workspace :

  `cd ~/catkin_ws`  
  `catkin_make`  
  `source devel/setup.bash`
    
+ Run the project with the following bash script provided inside the service_bot package:

  `./home_service.sh`
  
  Or run each node one by one manually in separated terminal instances.

## Known Issues
- Node crahes with error message: terminate called after throwing an instance of 'YAML::ParserException'
  what():  yaml-cpp: error at line 2, column 2: end of sequence not found

  Reason: Probably you have used tabs instead of spaces.
  Solution: convert all tabs to spaces (leading and non-leading).

## Resources:
- [Markers: Sending Basic Shapes (C++)](http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Basic%20Shapes)
- [YAML Ain’t Markup Language (YAML™) Version 1.2](https://yaml.org/spec/1.2/spec.html)
- [yaml-cpp Tutorial](https://github.com/jbeder/yaml-cpp/wiki/Tutorial)