#ifdef HAVE_SYS_TIME_H
#include <sys/time.h>
#endif

#include <boost/tokenizer.hpp>

#include <Eigen/Core>
#include <cmath>

#include <ros/console.h>

#include <pluginlib/class_list_macros.h>

#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>

#include <nav_core/parameter_magic.h>
#include <tf2/utils.h>

#include "dummy_local_planner.h"


const double INF                        = std::numeric_limits<double>::infinity();
const double EPSILON                    = std::numeric_limits<double>::epsilon();
const double PI                         = 3.141592653589793238463;

const double STEP_LINEAR_VELOCITY       = 0.01;
const double STEP_ANGULAR_VELOCITY      = 5/180.0 * PI;
const double MAX_LINEAR_VELOCITY        = 0.5;
const double MAX_ANGULAR_VELOCITY       = PI*50.0/180.0;
const double MAX_LINEAR_ACCELERATION    = 0.5;
const double MAX_ANGULAR_ACCELERATION   = PI*50.0/180.0;

const double DT                         = 0.1;

const double WEIGHT_DISTPLAN            = 0.1;
const double WEIGHT_HEADING             = 0.4;
const double WEIGHT_VELOCITY            = 0.0;

const int N_SAMPLES_TRAJ                = 30;
const int N_STEPS_AHEAD                 = 30;

const double FINE_POS_TOLERANCE         = 0.2;
const double XY_GOAL_TOLERANCE          = 0.01;
const double YAW_GOAL_TOLERANCE         = 10*PI/180;

const double ROBOT_LENGTH               = 0.665;
const double ROBOT_WIDTH                = 0.445;
bool ROT_STARTED                        = false;

ros::Time t,oldT;

using namespace std;



namespace dummy_local_planner {

  double currentVx;
  double currentVy;
  double currentWz;
  std::vector<geometry_msgs::PoseStamped> global_plan_;
  geometry_msgs::PoseStamped currentPose;
  struct ScoringHelper{
        double vx;
        double vy;
        double wz;
        double distPlan;
        double heading;
        double velocity;
        double score;
      };
  geometry_msgs::PoseStamped nearestPlanPose;
  geometry_msgs::PoseStamped errGoalPose;
  double sumDistPlan = 0;
  double sumHeading = 0;
  double sumVelocity = 0;

  void DummyLocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf,
      costmap_2d::Costmap2DROS* costmap_ros){
    if(!initialized_){
      
      prune_plan_ = false;
      xy_goal_tolerance_ = 0.1;
      yaw_goal_tolerance_ = 6.28;
      rot_stopped_vel_ = 1e-2;
      trans_stopped_vel_ = 1e-2;

      tf_ = tf;

      costmap_ros_ = costmap_ros;

      //to get odometry information, we need to get a handle to the topic in the global namespace
      // ros::NodeHandle gn;
      odom_sub_ = oh.subscribe<nav_msgs::Odometry>("odom", 1, boost::bind(&DummyLocalPlanner::odomCallback, this, _1));
      guiPathPub = nh.advertise<nav_msgs::Path>("plan", 1);

      initialized_ = true;
      std::cout<< "Planner Initialised " << std::endl;
    }
    else{
      ROS_WARN("This planner has already been initialized, doing nothing.");
    }
  }


  void DummyLocalPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    //we assume that the odometry is published in the frame of the base
    boost::mutex::scoped_lock lock(odom_mutex_);
    base_odom_.twist.twist.linear.x = msg->twist.twist.linear.x;
    base_odom_.twist.twist.linear.y = msg->twist.twist.linear.y;
    base_odom_.twist.twist.angular.z = msg->twist.twist.angular.z;
    ROS_DEBUG_NAMED("dummy_local_planner", "In the odometry callback with velocity values: (%.2f, %.2f, %.2f)",
      base_odom_.twist.twist.linear.x, base_odom_.twist.twist.linear.y, base_odom_.twist.twist.angular.z);

    currentVx = msg->twist.twist.linear.x;
    currentVy = msg->twist.twist.linear.y;
    currentWz = msg->twist.twist.angular.z;
  
  }


  // get velocities
  const double getLinearVelocityX() 
  {
    return currentVx;
  }

  const double getLinearVelocityY() 
  {
    return currentVy;
  }

  const double getAngularVelocityZ() 
  {
    return currentWz;
  }



  double clamp(double value, double minValue, double maxValue)
  {
    if(value < minValue)
      return minValue;
    if(value > maxValue)
      return maxValue;  
    return value;
  }


  // Check if there is an impact around the current location (each cell has the size specified in "resolution" in costmap_local_params.yaml)
  bool isImpactAroundLocation(geometry_msgs::PoseStamped currentPoseTemp)
  {
    unsigned int x, y, indX,indY;
    double yaw = tf::getYaw(currentPoseTemp.pose.orientation);

    costmap_2d::Costmap2D* costmap;
    costmap->worldToMap(currentPoseTemp.pose.position.x, currentPoseTemp.pose.position.y, x, y);

    double resolution = costmap->getResolution();

    for(int i = -(int)(ROBOT_LENGTH / resolution) ; i <= (int)(ROBOT_LENGTH / resolution) ; i++)
    {
      for(int j = -(int)(ROBOT_WIDTH / resolution) ; j <= (int)(ROBOT_WIDTH / resolution); j++)
      {
        indX = (unsigned int)clamp((x + (i*cos(yaw) - j*sin(yaw))),0,costmap->getSizeInCellsX());
        indY = (unsigned int)clamp((y + (i*sin(yaw) + j*cos(yaw))),0,costmap->getSizeInCellsY());
        if(costmap->getCost(indX, indY) != 0)
        {
          return true;
        }
      }
    }
    return false;
  }


  // get simulated robot pos
  geometry_msgs::PoseStamped getNewRobotPose(geometry_msgs::PoseStamped robotPose, double velocityVx, double velocityVy, double velocityWz)
  {
    geometry_msgs::PoseStamped newRobotPose;
    double robotYaw = tf::getYaw(robotPose.pose.orientation);
    newRobotPose.pose.position.x = robotPose.pose.position.x + (velocityVx*cos(robotYaw) - velocityVy*sin(robotYaw))*DT;
    newRobotPose.pose.position.y = robotPose.pose.position.y + (velocityVx*sin(robotYaw) + velocityVy*cos(robotYaw))*DT;
    newRobotPose.pose.position.z = 0;
    double newRobotYaw = robotYaw + velocityWz * DT;
    newRobotPose.pose.orientation = tf::createQuaternionMsgFromYaw(newRobotYaw);
    return newRobotPose;
  }

  bool DummyLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    //reset the global plan
    global_plan_.clear();
    global_plan_ = orig_global_plan;

    std::cout<< "Global Plan size is : " << global_plan_.size() << std::endl;

    return true;
  }

  geometry_msgs::PoseStamped getNewRobotGoal(geometry_msgs::PoseStamped robotPose)
  {
    int closestPointInPath = 0;
    double shortestDistance = INF;

    for(int i = 0; i < global_plan_.size(); i++)
    {
      double dx = global_plan_[i].pose.position.x - robotPose.pose.position.x;
      double dy = global_plan_[i].pose.position.y - robotPose.pose.position.y;
      double newDistance = sqrt(dx*dx + dy*dy);
      if(newDistance < shortestDistance)
      {
        shortestDistance = newDistance;
        closestPointInPath = i;
      }
    }
    if(closestPointInPath + N_STEPS_AHEAD > global_plan_.size()-1)
    {
      return global_plan_.back();
    }
    return global_plan_[closestPointInPath + N_STEPS_AHEAD];
  }

  // get heaading to goal - the value is inverted as tha cost function tries to maximize the returned value
  double getHeading(geometry_msgs::PoseStamped robotPose)
  {
    double angleToGoal = atan2(nearestPlanPose.pose.position.y - robotPose.pose.position.y, nearestPlanPose.pose.position.x - robotPose.pose.position.x );
    angleToGoal = angles::normalize_angle((angles::normalize_angle_positive(angleToGoal) - angles::normalize_angle_positive(tf::getYaw(robotPose.pose.orientation))));
    return 180 -  abs(angleToGoal)/PI * 180;
  }

  // get difference between two poses
  geometry_msgs::PoseStamped getPoseDifference(geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2)
  {
    geometry_msgs::PoseStamped diffPose;
    diffPose.pose.position.x = pose1.pose.position.x - pose2.pose.position.x;
    diffPose.pose.position.y = pose1.pose.position.y - pose2.pose.position.y;
    double tempYaw = tf::getYaw(pose1.pose.orientation) - tf::getYaw(pose2.pose.orientation);
    diffPose.pose.orientation = tf::createQuaternionMsgFromYaw(tempYaw);
    return diffPose;
  }


  // dwa algorithm - simulate trajectories usign N_SAMPLES_TRAJ samples
  bool getCommandsDWA(vector<ScoringHelper> &scoringhelper,double minVx,double maxVx,double minVy,double maxVy,double minWz,double maxWz)
  {
    for(double vx = minVx; vx <= maxVx; vx += STEP_LINEAR_VELOCITY)
    {
      for(double vy = minVy; vy <= maxVy; vy += STEP_LINEAR_VELOCITY)
      {
        for(double wz = minWz; wz <= maxWz; wz += STEP_ANGULAR_VELOCITY)
        {

          bool obstacleFound = false;
          geometry_msgs::PoseStamped currentPoseTemp = currentPose;

          for(int i = 0; i < N_SAMPLES_TRAJ; i++)
          {
            double oldX = currentPoseTemp.pose.position.x;
            double oldY = currentPoseTemp.pose.position.y;

            currentPoseTemp = getNewRobotPose(currentPoseTemp, vx, vy, wz);

            // if obstacl is found stop and do not use the velocity commands that would lead to that position
            if(isImpactAroundLocation(currentPoseTemp))
            {
              obstacleFound = true;
              break;
            }
          }
          if (!obstacleFound)
          {
            geometry_msgs::PoseStamped diffPlan = getPoseDifference(nearestPlanPose,currentPoseTemp);
            double distPlan = hypot(diffPlan.pose.position.x,diffPlan.pose.position.y);
            double heading = getHeading(currentPoseTemp);
            double velocity = hypot(vx,vy);

            sumDistPlan += distPlan;
            sumHeading += heading;
            sumVelocity += velocity;

            ScoringHelper sh;
            sh.vx = vx;
            sh.vy = vy;
            sh.wz = wz;
            sh.distPlan = distPlan;
            sh.heading = heading;
            sh.velocity = velocity;
            scoringhelper.push_back(sh);
          }
        }
      }
    }
    return true;
  }

  //  pid algorithm - used for final rotation
  bool getCommandsPID(geometry_msgs::Twist &cmd_vel, double err_x, double err_y, double err_th)
  {
    double th = tf::getYaw(currentPose.pose.orientation);
    double u_x_temp   = err_x;
    double u_y_temp   = err_y;
    double u_x   = cos(th) * u_x_temp + sin(th) * u_y_temp;
    double u_y   = -sin(th) * u_x_temp + cos(th) * u_y_temp;
    double u_th  = err_th ;

    cmd_vel.linear.x = clamp(u_x, -MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
    cmd_vel.linear.y = clamp(u_y, -MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
    cmd_vel.linear.z = 0;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = clamp(u_th, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);

    std::cout << " "<< cmd_vel.linear.x  << " " << cmd_vel.linear.y  << " " << cmd_vel.angular.z  << endl;


    return true;
  }




  bool DummyLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
    
    t = ros::Time::now() ;
    ros::Duration delta_time = (t-oldT);
    oldT = t;
    
    if(!initialized_){
      std::cout<< "This planner has not been initialized, please call initialize() before using this planner \n";
      return false;
    }

    // geometry_msgs::PoseStamped global_pose;
    if(!costmap_ros_->getRobotPose(currentPose)){
      std::cout<< "Could not get robot pose\n";
      return false;
    }

    vector<ScoringHelper> scoringhelper;
    nearestPlanPose = getNewRobotGoal(currentPose);

    costmap_2d::Costmap2D* costmap;
    costmap = costmap_ros_->getCostmap();
    costmap_ros_->getRobotPose(currentPose);


    double robotVx = getLinearVelocityX();
    double robotVy = getLinearVelocityY();
    double robotWz = getAngularVelocityZ();

        // get velocity ranges
    double minVx = clamp(robotVx - DT*MAX_LINEAR_ACCELERATION, -MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
    double maxVx = clamp(robotVx + DT*MAX_LINEAR_ACCELERATION, -MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
    double minVy = clamp(robotVy - DT*MAX_LINEAR_ACCELERATION, -MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
    double maxVy = clamp(robotVy + DT*MAX_LINEAR_ACCELERATION, -MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
    double minWz = clamp(robotWz - DT*MAX_ANGULAR_ACCELERATION, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
    double maxWz = clamp(robotWz + DT*MAX_ANGULAR_ACCELERATION, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);

    std::cout << hypot(errGoalPose.pose.position.x, errGoalPose.pose.position.y) << std::endl;
    if (!hypot(errGoalPose.pose.position.x, errGoalPose.pose.position.y) > FINE_POS_TOLERANCE)
    {

      std::cout<< "inside DWA control "<<std::endl;
      // get velocity commands using the dwa algorithm
      sumDistPlan = 0;
      sumHeading = 0;
      sumVelocity = 0;
      getCommandsDWA(scoringhelper, minVx, maxVx, minVy, maxVy, 0.0, 0.0);
      getCommandsDWA(scoringhelper, minVx, maxVx, 0.0, 0.0, minWz, maxWz);
      //getCommandsDWA(scoringhelper, 0.0, 0.0, minVy, maxVy, minWz, maxWz);

      // find velocity commands that maximizes the score
      int bestScoreIndex = 0;
      for(int i = 0; i < scoringhelper.size(); i++)
      {
        double normalized_distPlan  = scoringhelper[i].distPlan / sumDistPlan;
        double normalized_heading   = scoringhelper[i].heading / sumHeading;
        double normalized_velocity  = scoringhelper[i].velocity / sumVelocity;

        scoringhelper[i].score =  WEIGHT_DISTPLAN / normalized_distPlan  + WEIGHT_HEADING * normalized_heading + WEIGHT_VELOCITY * normalized_velocity;
        if(scoringhelper[i].score > scoringhelper[bestScoreIndex].score)
        {
          bestScoreIndex = i;
        }
      }
      if(scoringhelper.size() == 0)
      {
        cout << "Failed finding velocity comands" << endl;
        return false;
      }
      else
      {
        cmd_vel.linear.x = scoringhelper[bestScoreIndex].vx;
        cmd_vel.linear.y = scoringhelper[bestScoreIndex].vy;
        cmd_vel.linear.z = 0;
        cmd_vel.angular.x = 0;
        cmd_vel.angular.y = 0;
        cmd_vel.angular.z = scoringhelper[bestScoreIndex].wz;
      }
    }
    else
    {
      // std::cout<< "inside PID control "<<std::endl;
      // get PID velocity commands for final positioning
      if ((hypot(errGoalPose.pose.position.x, errGoalPose.pose.position.y) > XY_GOAL_TOLERANCE) && !ROT_STARTED)
      {
        getCommandsPID(cmd_vel, errGoalPose.pose.position.x, errGoalPose.pose.position.y, 0);
      }
      else
      {
        // get PID velocity commands for final rotation
        getCommandsPID(cmd_vel, 0,0, tf::getYaw(errGoalPose.pose.orientation));
        // if the final rotation is initiatd then complete it - to avoid toggling between different states
        if(fabs(tf::getYaw(errGoalPose.pose.orientation)) <= YAW_GOAL_TOLERANCE)
        {
          ROT_STARTED = false;
        }
        else
        {
          ROT_STARTED = true;
        }
      }
    }

    return true;
  }


  bool DummyLocalPlanner::isGoalReached(){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    geometry_msgs::PoseStamped global_pose;
    if(!costmap_ros_->getRobotPose(global_pose)){
      ROS_ERROR("Could not get robot pose");
      return false;
    }

    //copy over the odometry information
    nav_msgs::Odometry base_odom;
    {
      boost::mutex::scoped_lock lock(odom_mutex_);
      base_odom = base_odom_;
    }

    return base_local_planner::isGoalReached(*tf_, global_plan_, *(costmap_ros_->getCostmap()), costmap_ros_->getGlobalFrameID(), global_pose, base_odom, 
        rot_stopped_vel_, trans_stopped_vel_, xy_goal_tolerance_, yaw_goal_tolerance_);
    
  }

};

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(dummy_local_planner::DummyLocalPlanner, nav_core::BaseLocalPlanner)
