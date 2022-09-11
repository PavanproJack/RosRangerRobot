# RosRangerRobot

RangerRobot is a ROS package that implements a new local planner for move_base which is a ROS interface for interacting with the navigation stack on a robot. 

This project aims to autonomously navigate a robot from it's source location to the target location specifically in a ware house environment scene.

## Components and Software
RangerRobot is built with the following:

1. `ROS1` on melodic version
2. [`TurtleBot3 burger`](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/), ROS-based mobile robot
3. [`Gazebo`](https://classic.gazebosim.org/) multi-robot simulator on version 9.0.0


## Custom local planner using ROS Navigation Stack
This repository contains a custom Local planners.
The algorithms implemented are
- DWA for local planner

The following functions from the **BaseLocalPlanner** interface are overriden in the custom local planner:
- initialize
- computeVelocityCommands
- isGoalReached
- setPlan

## Build and Running Instructions

```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/

$ catkin build     
$ source devel/setup.bash  
```
And for 

Navigation with the `BaseGlobalPlanner` and `EbandLocalPlanner` enable the following in move_base.launch under the `launch`
folder.

```
    <param name="base_global_planner" value="global_planner/GlobalPlanner" />
    <param name="base_local_planner" value="eband_local_planner/EBandPlannerROS"/>
```

and for launch with the customised local planner

```
    <param name="base_global_planner" value="global_planner/GlobalPlanner" />
    <param name="dummy_local_planner" value="dummy_local_planner/DummyLocalPlanner" />
```

With these changes, hit this command on the terminal

```
$ roslaunch ranger_bot turtlebot3_navigation.launch
```

This opens up RVIZ and Gazebo world ....... 


### Assigning Tasks/Goals

#### **Two ways**:
* Click the 2D Nav Goal button in the RViz menu on Toolbar.Set the destination of the robot by dragging the green arrow toward the direction where the robot will be facing. OR

* Launch the `goal_handler` node like this and follow the instructions on terminal.
```
$  rosrun ranger_bot goal_handler 
```

Launching `goal_handler` node also helps us in tracking if the robot is following the calculated path or if it is lost/stuck by publishing the robot status on the ros-topic `/robot_status`. Check this out by running the following on a sourced terminal.

```
$ rostopic echo /robot_status
```

## Note
***Getting the robot to navigate with the custom local planner is still a work inProgress while different approaches are experimented.***


## References

1. Environment Model: [AWS RoboMaker Small Warehouse World](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world)

2. Mapping the environment: [Gmapping SLAM method](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam_simulation/)

3. Navigating robot with base global and local planners: [Mastering ROS for Robotics Programming Book](https://books.google.co.uk/books?hl=en&lr=&id=MulODwAAQBAJ&oi=fnd&pg=PP1&dq=Mastering+ROS+for+Robotics+Programming&ots=Clm6JZl-qP&sig=T8PgJ2vYnygKOkXfwT9QBLTOZak&redir_esc=y#v=onepage&q=Mastering%20ROS%20for%20Robotics%20Programming&f=false)

4. Creating custom global and local planners: [Global and Local planner Plugins ROS](http://wiki.ros.org/navigation/TutorialsWriting%20A%20Global%20Path%20Planner%20As%20Plugin%20in%20ROS)

5. [BaseLocalPlanner interface](https://github.com/ros-planning/navigation/tree/4a3d261daa4e7eafa40bf7e4505f8aa8678d7bd7/base_local_planner) and [DWALocalPlanner interface](https://github.com/ros-planning/navigation/tree/4a3d261daa4e7eafa40bf7e4505f8aa8678d7bd7/dwa_local_planner)

6. [`amcl`](http://wiki.ros.org/amcl) ros package which is a probabilistic localization system for a robot moving in 2D

7. Others:  [Relaxed Astar](https://github.com/aranyadan/relaxed_astar), [Bubble local Planner](https://github.com/adrianapadilla/bubble_local_planner) for obstacle avoidance, customised [DWA local planner](https://github.com/davidezilio/custom_navigation), [Pure-pursuit local planner](https://github.com/raphaelkba/pure_pursuit)

