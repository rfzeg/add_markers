# Abstract
ROS node that reads in Rviz marker positions from a YAML file and publishes them to Rviz as required.

# Repository architecture 
## Source code files :
+ **add_markers.cpp** : load positions of Rviz Markers from YAML file and publish them when called in main().
 
## Directories :
+ **config/** : contains YAML file that stores the position and orientation of Rviz markers.
+ **rviz/** : contains Rviz configuration that includes a subscriber to the Rviz marker topic.
+ **src/** : contains source code, **add_markers.cpp**.

# Usage example using Gazebo as simulation:

+ Clone following repositories to a catkin workspace (for example ~/catkin_ws/src):

  `git clone https://github.com/rfzeg/service_bot.git`  
  `git clone https://github.com/rfzeg/dumpster.git`  
  `git clone https://github.com/rfzeg/navi_goals.git`  
  `git clone https://github.com/rfzeg/add_markers.git`

+ If not already present, install xterm:

  `sudo apt-get install xterm`
  
+ Build and source your workspace :

  `cd ~/catkin_ws`  
  `catkin_make`  
  `source devel/setup.bash`
    
+ Run the project with a bash script provided inside the service_bot package:

  `./home_service.sh`
  
