################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.0.2)
project(ranger_bot)
cmake_policy(SET CMP0054 NEW)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
  gazebo_ros
  nav_core
  pluginlib
  nav_msgs
  costmap_2d
  base_local_planner
  dynamic_reconfigure
  pcl_conversions
)

find_package(gazebo REQUIRED)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters

################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs nav_msgs tf gazebo_ros nav_core dynamic_reconfigure pluginlib
  DEPENDS GAZEBO
)

################################################################################
# Build
################################################################################
link_directories(${GAZEBO_LIBRARY_DIRS})

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_definitions(${Eigen3_DEFINITIONS})

link_directories(${catkin_LIBRARY_DIRS})

add_executable(turtlebot3_drive src/turtlebot3_drive.cpp)
add_dependencies(turtlebot3_drive ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(turtlebot3_drive ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(goal_handler src/goal_handler.cpp)
add_dependencies(goal_handler ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(goal_handler ${catkin_LIBRARIES})


# add_library(RAstar_planner_lib src/path_planners/global_planners/RAstar_planner.cpp)
# target_link_libraries(
#      RAstar_planner_lib 
#      ${catkin_LIBRARIES} 
# )


# add_library(simple_local_planner_lib src/path_planners/local_planners/simple_local_planner.cpp)
# target_link_libraries(simple_local_planner
#    ${catkin_LIBRARIES}
#  )


add_library(dummy_local_planner_lib src/path_planners/local_planners/dummy_local_planner.cpp)
target_link_libraries(dummy_local_planner_lib ${catkin_LIBRARIES})


# add_library(pure_pursuit_local_planner_lib src/path_planners/local_planners/pure_pursuit_planner.cpp src/path_planners/local_planners/transform_global_plan.cpp src/path_planners/local_planners/join_costmap.cpp)
# target_link_libraries(pure_pursuit_local_planner_lib ${catkin_LIBRARIES})
# add_dependencies(pure_pursuit_local_planner_lib pure_pursuit_local_planner_gencfg)
# add_dependencies(pure_pursuit_local_planner_lib nav_msgs_gencpp)


catkin_install_python(PROGRAMS src/init_pose.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

################################################################################
# Install
################################################################################
install(TARGETS turtlebot3_drive
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch models rviz worlds 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# install(TARGETS RAstar_planner_lib
#   # ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   # RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

install(TARGETS dummy_local_planner_lib
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )

install(FILES dummy_local_planner.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)



# install(DIRECTORY config/
#    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
# )

################################################################################
# Test
################################################################################
