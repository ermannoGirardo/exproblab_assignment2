   
/**
* \file check_hypothesis.cpp
* \brief check hypothesis acquired
* \author Ermanno Girardo
* \version 1.0
* \date 15/04/2022
*
* Clients : <BR>
*   /test_consistency
*
*   /replan_srv
*
* Description :
*
* This node check the acquired hypothesis to test its consistency.
* If no one hypothesis is consistent trigger replanning
*
*/

#include "my_rosplan_interface/check_hypothesis.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <erl2/go_to_pointAction.h>
#include <erl2/JointsArmGoal.h>
#include <erl2/ConsistentHypothesis.h>
#include <erl2/Replan.h>

//GLOBAL VARIABLES
//Client for /test_consistency
ros::ServiceClient  consistent_hp_client;

//Client for /replan_service
ros::ServiceClient replan_client;

//Custom srv for test consistency
erl2::ConsistentHypothesis cons_hp_srv;

//Function to see if at least one hp is consistent:

bool is_one_consistent(){
	bool consistent_hp = false;
	for (int i = 0; i<6;i++){
		if(cons_hp_srv.response.consistent_IDs[i]){
			consistent_hp = true;
			std::cout << "ID:" << i << "is consistent" << std::endl;
		}
	}
	return consistent_hp;
}




namespace KCL_rosplan {
	CheckHypothesisActionInterface::CheckHypothesisActionInterface(ros::NodeHandle &nh) {
		// here the initialization
	}
	bool CheckHypothesisActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
		// here the implementation of the action
		std::cout << "Check if there are consistent hp" << std::endl << "If no force replan!!!" << std::endl;
		//Set the new_id request to true:
		cons_hp_srv.request.new_hp = true;
		//Makes the client call to update the consistent hp
		consistent_hp_client.call(cons_hp_srv);
		//See if there is at least one consistent hp
		bool cons_hp = is_one_consistent();
		erl2::Replan replan_srv;
		if (!cons_hp){
			std::cout << "There are no consistent hypotheses!" << "Force replanning" << std::endl;
			replan_client.call(replan_srv);
		}
		else{
			std::cout << "At least one hp is consistent"  << "        " << "Go to test the hypothesis" << std::endl;
		}
		ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
		return true;
	}
}


int main(int argc, char **argv) {
ros::init(argc, argv, "my_rosplan_action_check_hypothesis", ros::init_options::AnonymousName);
ros::NodeHandle nh("~");
consistent_hp_client = nh.serviceClient<erl2::ConsistentHypothesis>("/test_consistency");
replan_client = nh.serviceClient<erl2::Replan>("/replan_service");
KCL_rosplan::CheckHypothesisActionInterface my_aci(nh);
my_aci.runActionInterface();
return 0;
}





