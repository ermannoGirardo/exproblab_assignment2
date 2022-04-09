/**
* \file initialization.cpp
* \brief extend the arm at first and then move the robot
* \author Ermanno Girardo
* \version 1.0
*
*  Clients : <BR>
*    /move_arm_service
*
* ActionClients : <BR>
*    /go_to_point
*
* Description :
*
* This node has the purpose of initialize the investigation.
* In particular at start it extends the arm and then move the robot to wp.
*
*/

#include "my_rosplan_interface/initialization.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
//#include <motion_plan/PlanningAction.h>
#include <erl2/go_to_pointAction.h>
#include <erl2/JointsArmGoal.h>

// GLOBAL VARIABLES
ros::ServiceClient move_arm_client;

namespace KCL_rosplan {
	InitializationActionInterface::InitializationActionInterface(ros::NodeHandle &nh) {
		// here the initialization
		
	}
	bool InitializationActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
		// here the implementation of the action
		actionlib::SimpleActionClient<erl2::go_to_pointAction> ac("/go_to_point", true);
		erl2:: JointsArmGoal arm_goal;
		//Configuration with the arm extended:
		std::cout << "Extend the arm" << std::endl;
		arm_goal.request.joint0 = -3.14/2;
		arm_goal.request.joint1 = 0;
		arm_goal.request.joint2 = 0;
		arm_goal.request.joint3 = 0;
		arm_goal.request.joint4 = 0;
		//-*//-*/-/-*-//-*-//-*-//-*-//-*-//
		move_arm_client.call(arm_goal);
 		std::cout << "Going to " << msg->parameters[0].value << std::endl;
		erl2:: go_to_pointGoal goal;
		ac.waitForServer();
		if(msg->parameters[0].value == "wp1"){
			goal.x = 2;
			goal.y = 0.0;
			goal.theta = 0;
		}
		else if (msg->parameters[0].value == "wp2"){
			goal.x = 0;
			goal.y = 2;
			goal.theta = 3.14/2;
		}
		else if (msg->parameters[0].value == "wp3"){
			goal.x = -2;
			goal.y = 0;
			goal.theta = 3.14;
		}
		else if (msg->parameters[0].value == "wp4"){
			goal.x = 0;
			goal.y = -2;
			goal.theta = 3.14*3/2;
		}
		ac.sendGoal(goal);
		ac.waitForResult();
		ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
		return true;
	}
}

/// Initialize the investigation
int main(int argc, char **argv) {
ros::init(argc, argv, "my_rosplan_action_initialize", ros::init_options::AnonymousName);
ros::NodeHandle nh("~");
move_arm_client = nh.serviceClient<erl2::JointsArmGoal>("/move_arm_service");
KCL_rosplan::InitializationActionInterface my_aci(nh);
my_aci.runActionInterface();
return 0;
}
