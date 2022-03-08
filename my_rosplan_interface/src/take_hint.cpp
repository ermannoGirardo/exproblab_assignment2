/**
* \file take_hint.cpp
* \brief  This is the client for moving the robot arm
* \author Ermanno Girardo
* \version 1.0
*
*  Clients : <BR>
*    /move_arm_client
* Description :
*
* This client want to move the robot arm specifing the values for each joint.
* The motion will be provided implementing a service to move the arm thanks to moveit pkg
*
*/

#include "my_rosplan_interface/take_hint.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
//#include <motion_plan/PlanningAction.h>
#include <erl2/go_to_pointAction.h>
#include <erl2/JointsArmGoal.h>

//Glogal Variables:
ros::ServiceClient move_arm_client;


namespace KCL_rosplan {
	TakeHintActionInterface::TakeHintActionInterface(ros::NodeHandle &nh) {
		// here the initialization
		
	}
	bool TakeHintActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
		// here the implementation of the action
		std::cout << "I am taking the hint of: " << msg->parameters[0].value << std::endl;
		//In order to stop the arm shaking wait 2 sec
		usleep(2000000); 
		erl2:: JointsArmGoal goal;
		goal.request.joint0 = -3.14/2;
		goal.request.joint1 = -0.5;
		goal.request.joint2 = -3.14/2;
		goal.request.joint3 = 0;
		move_arm_client.call(goal);
		goal.request.joint1 = 0;
		goal.request.joint2 = 0;
		move_arm_client.call(goal);

		ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
		return true;
	}
}



/// This node move the arm of the robot to reach the hint
int main(int argc, char **argv) {
ros::init(argc, argv, "my_rosplan_action_take_hint", ros::init_options::AnonymousName);
ros::NodeHandle nh("~");
move_arm_client = nh.serviceClient<erl2::JointsArmGoal>("/move_arm_service");
KCL_rosplan::TakeHintActionInterface my_aci(nh);
my_aci.runActionInterface();
return 0;
}
