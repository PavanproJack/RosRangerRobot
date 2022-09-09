#include "local_planner/local_planner.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(local_planner, LocalPlanner, local_planner::LocalPlanner, nav_core::BaseLocalPlanner)

namespace local_planner{

        LocalPlanner::LocalPlanner() : costmap_ros_(NULL), tf_(NULL), initialized_(false) {}

        LocalPlanner::LocalPlanner(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
         : costmap_ros_(NULL), tf_(NULL), initialized_(false)
         {
                initialize(name, tf, costmap_ros);
         }

        LocalPlanner::~LocalPlanner() {}

        void LocalPlanner::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
        {

                if(!initialized_)
                {
        }

        }

        bool LocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
        {

                if(!initialized_)
                {
                ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
                return false;
                }

                return true;
        }

        bool LocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
        {

                if(!initialized_)
                {
                    ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
                    return false;
                }

                return true;

        }

        bool LocalPlanner::isGoalReached()
        {
                if(!initialized_)
                {
                        ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
                        return false;
                }

                return false;
        }
}