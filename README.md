# RosRangerRobot

RangerRobot is a ROS package that implements a new local planner for move_base which is a ROS interface for interacting with the navigation stack on a robot. 

# Components and Software
RangerRobot is built with the following:

1. `ROS1` on melodic version
2. [`TurtleBot3`](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/), ROS-based mobile robot
3. [`Gazebo`](https://classic.gazebosim.org/) multi-robot simulator on version 9.0.0

1. [`amcl`](http://wiki.ros.org/amcl) ros package which is a probabilistic localization system for a robot moving in 2D
2. 

# Custom local planner using ROS Navigation Stack
This repository contains a custom Local planners.
The algorithms implemented are
- DWA for local planner

Both planners have been developed following the specificied interfaces
- [Tutorial](http://wiki.ros.org/navigation/Tutorials/Writing%20a%20Local%20Path%20Planner%20As%20Plugin%20in%20ROS)

- [BaseLocalPlanner interface](http://docs.ros.org/melodic/api/nav_core/html/classnav__core_1_1BaseLocalPlanner.html)


The following functions from the **BaseLocalPlanner** interface are overriden in the custom local planner:
- initialize
- computeVelocityCommands
- isGoalReached
- setPlan

# Running Instructions
`roslaunch ranger_bot turtlebot3_navigation.launch`
