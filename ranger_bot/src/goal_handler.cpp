#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include "std_msgs/String.h"

// using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class goalHandler{
    private:
        int input;
        char choice;
        move_base_msgs::MoveBaseGoal goal;
        bool valid_selection;
        bool run;
        double connection_timeout;
        ros::Publisher robot_status_publisher;
        std_msgs::String msg;
    
    public:
        void chooseJob();
        goalHandler(ros::NodeHandle* nh): input(1), choice('Y'), run(true), connection_timeout(5.0) {
          robot_status_publisher = nh->advertise<std_msgs::String>("/robot_status", 10); 
          chooseJob();
        }
};

void goalHandler::chooseJob(){

  MoveBaseClient MB("move_base", true);

  /**
   * \brief Waits for the ActionServer to connect to this client
   * \param timeout Max time to block before returning. A zero timeout is interpreted as an infinite timeout.
   * \return True if the server connected in the allocated time. False on timeout
   */
  while(!MB.waitForServer(ros::Duration(goalHandler::connection_timeout))){
              ROS_INFO("Waiting for the move_base action server to come up");
  }

  /**
   * @brief  Checks if the action client is successfully connected to the action server
   * @return True if the server is connected, false otherwise
   */
  while(goalHandler::run  && MB.isServerConnected()){

    std::cout << "Action Client successfully to the action server \n";

    std::cout << "\nWonderful. There are three pre-defined destinations as listed here. Please select one of them" << std::endl;
    std::cout << "\n1 for Rack 1" << std::endl;
    std::cout << "2 for Rack 2" << std::endl;
    std::cout << "3 for Rack 3" << std::endl;
    
    std::cin >> goalHandler::input;

    std::cout<<"\n\nGreat! You chose to navigate to Rack" <<goalHandler::input << "\n\nNavigating now ...."<< std::endl;
 
    // Send a goal to the robot
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goalHandler::valid_selection = true;

  /*
   * Let the user command goals to the turtlebot by choosing the destinations
  */

    switch (goalHandler::input) {
      case 1:
        std::cout << "\nGoal Location: 1\n" << std::endl;
        goal.target_pose.pose.position.x = 1.7462;
        goal.target_pose.pose.position.y = -7.7118;
        goal.target_pose.pose.orientation.w = 1.0;
        break;

      case 2:
        std::cout << "\nGoal Location: 2\n" << std::endl;
        goal.target_pose.pose.position.x = 1.64537;
        goal.target_pose.pose.position.y = 4.3;
        goal.target_pose.pose.orientation.w = 1.0;
        break;

      case 3:
        std::cout << "\nGoal Location: 3\n" << std::endl;
        goal.target_pose.pose.position.x = -1.6228;
        goal.target_pose.pose.position.y = -1.6228;
        goal.target_pose.pose.orientation.w = 1.0;
        break;

      default:
        std::cout << "\nInvalid selection. Please try again.\n" << std::endl;
        goalHandler::valid_selection = false;
    }

    if(!valid_selection) {
      continue;
    }

  /**
   * \brief Sends a goal to the ActionServer, and also registers callbacks
  */
    MB.sendGoal(goal);

      /**
   * \brief Sends a goal to the ActionServer,
   * and waits until the goal completes or a timeout is exceeded
   */
    // MB.sendGoalAndWait(goal, 50.0)


  /**
   * \brief Blocks until this goal finishes
   * \param timeout Max time to block before returning. A zero timeout is interpreted as an infinite timeout.
   * \return True if the goal finished. False if the goal didn't finish within the allocated timeout
   */
    MB.waitForResult();


  /**
   * \brief Get the state information for this goal
   *
   * Possible States Are: PENDING, ACTIVE, RECALLED, REJECTED, PREEMPTED, ABORTED, SUCCEEDED, LOST.
   * \return The goal's state. Returns LOST if this SimpleActionClient isn't tracking a goal.
   * 
   * Also publishes the status of the robot for tracking.
   */
    goalHandler::msg.data = " ";
    if(MB.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("Success. Robot has reached the assigned destination");
      goalHandler::msg.data = "SUCCESS";
    }  
    else if(MB.getState() == actionlib::SimpleClientGoalState::LOST){
      goalHandler::msg.data = "LOST";
    }
    else if(MB.getState() == actionlib::SimpleClientGoalState::PENDING){
      goalHandler::msg.data = "PENDING";
    }
    else if(MB.getState() == actionlib::SimpleClientGoalState::ACTIVE){
      goalHandler::msg.data = "ACTIVE";
    }
    else{
      ROS_INFO("OOPs, Failed to reach the assigned destination.");
      goalHandler::msg.data = "FAILURE";
    }
    
    goalHandler::robot_status_publisher.publish(goalHandler::msg);
         
    do {
      std::cout << "\nWould you like add another destination? (Y/N)" << std::endl;
      std::cin >> goalHandler::choice;;
      goalHandler::choice = tolower(goalHandler::choice); 
    } while (goalHandler::choice != 'n' && goalHandler::choice != 'y'); 
 
    if(goalHandler::choice =='n') {
        goalHandler::run = false;
        exit(0);
    }
  }



}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "goal_handler");
    ros::NodeHandle NH;
    goalHandler GH = goalHandler(&NH);
    ros::spin();
    return 0;
}