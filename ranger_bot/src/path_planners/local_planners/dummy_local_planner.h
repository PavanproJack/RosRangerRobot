#ifndef dummy_local_planner_ROS_H_
#define dummy_local_planner_ROS_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <tf/tf.h>

#include <angles/angles.h>
//#include <boost/shared_ptr.hpp>
#include <nav_core/base_local_planner.h>

namespace dummy_local_planner {

  class DummyLocalPlanner : public nav_core::BaseLocalPlanner {
    public:

      DummyLocalPlanner() : costmap_ros_(NULL), tf_(NULL), initialized_(false) {}

      /**
       * @brief  Constructs the ros wrapper
       * @param name The name to give this instance of the trajectory planner
       * @param tf A pointer to a transform listener
       * @param costmap The cost map to use for assigning costs to trajectories
       */
      void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief  Check if the goal pose has been achieved
       * @return True if achieved, false otherwise
       */
      bool isGoalReached();

      /**
       * @brief  Set the plan that the controller is following
       * @param orig_global_plan The plan to pass to the controller
       * @return True if the plan was updated successfully, false otherwise
       */
      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

      /**
       * @brief  Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
       * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
       * @return True if a valid trajectory was found, false otherwise
       */
      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

    private:

      /**
       * @brief  Callback for receiving odometry data
       * @param msg An Odometry message 
       */
      void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);


      /**
       * @brief Compute the square distance between two poses
       */

      costmap_2d::Costmap2DROS* costmap_ros_;
      tf2_ros::Buffer* tf_;
      double rot_stopped_vel_, trans_stopped_vel_;
      double yaw_goal_tolerance_, xy_goal_tolerance_;
      bool prune_plan_;
      bool initialized_;
      ros::Subscriber odom_sub_;
      boost::mutex odom_mutex_;
      nav_msgs::Odometry base_odom_;
      std::vector<geometry_msgs::PoseStamped> global_plan_;
  };
};
#endif