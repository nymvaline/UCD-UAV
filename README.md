# Project UAV at UCDavis.
This is repo for custom applications in ROS.  
We are using PX4 flight stack with ROS, MAVROS and Edison as companion computer.  
Note the repo is mainly updated on TCS series.  
TCS refers to Task Control Software, which is task scheduler/manager for Tasks.  
Task is broad concept about things that UAV can do, including moving, data collecting, collision avoidance, etc.  

This repository is a ROS package.  
In order to compile and use this:  
+ wiki.ros.org/ROS/Installation  
+ You will also need to install mavros ( http://dev.px4.io/ros-mavros-installation.html ), we are currently using kinetic.  
+ Not sure about ROS version compatibility, will update that later.  
This is compiled and managed using catkin.  
To use this package,  
+ http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment  
+ Place this file underneath src in your newly-created catkin environment.  
+ catkin_make  
