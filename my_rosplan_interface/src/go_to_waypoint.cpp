/**
* \file goto_waypoint.cpp
* \brief move my robot
* \author Ermanno Girardo
* \version 1.0
*
* ActionClients : <BR>
*    /go_to_point
*
* Description :
*
* move the robot to the correct waypoint (the one choosen by the planner)
*
*/

#include "my_rosplan_interface/go_to_waypoint.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
//#include <motion_plan/PlanningAction.h>
#include <erl2/go_to_pointAction.h>

// GLOBAL VARIABLES


namespace KCL_rosplan {
  GoToWaypointActionInterface::GoToWaypointActionInterface(ros::NodeHandle &nh) {
    // here the initialization

  }
  bool GoToWaypointActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
    // here the implementation of the action
    std::cout << "Going from: " << msg->parameters[0].value << " to: "<< msg->parameters[1].value << std::endl;
    actionlib::SimpleActionClient<erl2::go_to_pointAction> ac("/go_to_point", true);
    
    erl2:: go_to_pointGoal goal;
    ac.waitForServer();
    if(msg->parameters[1].value == "wp0"){
      goal.x = 0.0;
      goal.y = 0.0;
      goal.theta = 0;
    }
    if(msg->parameters[1].value == "wp1"){
      goal.x = 2.5;
      goal.y = 0.0;
      goal.theta = 0;
    }
    else if (msg->parameters[1].value == "wp2"){
      goal.x = 0;
      goal.y = 2.5;
      goal.theta = 3.14/2;
    }
    else if (msg->parameters[1].value == "wp3"){
      goal.x = -2.5;
      goal.y = 0;
      goal.theta = 3.14;
    }
    else if (msg->parameters[1].value == "wp4"){
      goal.x = 0;
      goal.y = -2.5;
      goal.theta = 3.14*3/2;
    }
    
    //std::cout << "Debug" <<std::endl;
    ac.sendGoal(goal);
    ac.waitForResult();
    ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
    return true;
  }
}

///This node move the robot to the correct waypoint
int main(int argc, char **argv) {
ros::init(argc, argv, "my_rosplan_action_goto_waypoint", ros::init_options::AnonymousName);
ros::NodeHandle nh("~");
KCL_rosplan::GoToWaypointActionInterface my_aci(nh);
my_aci.runActionInterface();
return 0;
}
