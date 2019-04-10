# add_markers

## Abstract
ROS node that reads in Rviz marker positions from a YAML file and publishes them to Rviz as required.

## Repository architecture 
### Source code files :
+ **add_markers.cpp** : load Rviz Markers from YAML file and publish them when called in main()
 
### Directories :
+ **src/** : (required) contains source code, **add_markers.cpp**
+ **config/** : (required) contains YAML file that stores the position and orientation of markers
+ **rviz/** : (optional) contains Rviz configuration file ready for displaying visual markers
+ **launch/** : (optional) contains launch file for the executable file of this package

## YAML file structure

A `markers.yaml` file should look like this:

```yml
markers:
  - point:
      x: 1.0
      y: 3.0
      th: 0 # in degrees
  - point:
      x: 3.5
      y: 2.5
      th: 90 # in degrees
```

As showed above the marker's definition file `markers.yaml` has the following structure:

- `markers` (required): root object that identifies the YAML file as containing Rviz markers data
- `point` (required): block key that groups the definition of each marker's position and orientation in the map  
- `x`, `y` and `th` (required): key-value pairs that contain the coordinates (x,y) and orientation (th), in degrees, of each marker

It is possible to add comments to the YAML file by using the # sign. All `point` elements begin with a dash (-) and must be prefixed with the same amount of spaces, in the example above two spaces are used, the number of spaces can vary from file to file, but tabs are not allowed. In like manner the `x`, `y` and `th` keys also require the same amount of spaces in front of each (at least as many spaces as each `point` key has).  
  
The YAML file can be adjusted by adding or deleting markers, changing they order and adding/removing comments. It is important to do not break the formatting rules described above while editing the file or the markers will not load properly.

## Direct usage:

- Clone this repository into a ROS catkin workspace
- Build and source the workspace
- To launch this package including Rviz: `roslaunch add_markers add_markers.launch`


## Usage example in combination with a waypoint generator node using Gazebo as simulation:

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
  
  Or run each node one by one manually in separated terminal instances
