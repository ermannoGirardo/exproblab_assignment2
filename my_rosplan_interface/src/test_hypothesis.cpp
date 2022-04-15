/**
* \file test_hypothesis.cpp
* \brief test an hypothesis going to the oracle
* \author Ermanno Girardo
* \version 1.0
* \date 15/04/2022
*
* Clients : <BR>
*   /oracle_solution
*   /test consistency
*
*   ActionClients : <BR>
*    /go_to_point
* 
*   Publisher : <BR>
*   /says_hp
*
* Description :
*
* Once an hypothesis is consistent this node action first force the robot to
* go in the centre of the arena (oracle room) and then compare the ID of the 
* consistent hypothesis to test its truthfulness.
* If the hypothesis is wrong then cancel it.
* 
*
*/


#include "my_rosplan_interface/test_hypothesis.h"
#include <unistd.h>
#include <stdio.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <erl2/go_to_pointAction.h>
#include <erl2/JointsArmGoal.h>
#include <erl2/ConsistentHypothesis.h>
#include <erl2/Replan.h>
#include <erl2/Oracle.h>
#include "std_msgs/Int32.h"

//Global Variables//
//Service clients
ros::ServiceClient cons_hp_client;
ros::ServiceClient test_hp_client;

int solution_ID = -1; //Init as not found
std_msgs::Int32  say_hp_ID;

//Custom srv for test consistency
erl2::ConsistentHypothesis cons_hp_srv;

//Custom srv for test thruthfulness hp
erl2::Oracle oracle_ID;

//Publisher to says_hp
ros::Publisher solution_pub;

namespace KCL_rosplan {
  TestHypothesisActionInterface::TestHypothesisActionInterface(ros::NodeHandle &nh) {
    // here the initialization
  }
	bool TestHypothesisActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
		// here the implementation of the action
		std::cout << "New consistent hypotheses collected, let's go to the oracle" << std::endl;
		//Initialize the action client for go_to_point action
		actionlib::SimpleActionClient<erl2::go_to_pointAction> action_client("/go_to_point", true);
		//Initialize the goal
		erl2:: go_to_pointGoal cluedo_room;
		action_client.waitForServer();
		cluedo_room.x = 0;
		cluedo_room.y = 0;
		cluedo_room.theta = 0;
		//Send the goal 
		action_client.sendGoal(cluedo_room);
		//Wait for the result
		action_client.waitForResult();
		std::cout << "Arrived to the oracle room" << std::endl;
		
		//Ask for consistent hp:
		cons_hp_srv.request.new_hp = false;
		cons_hp_client.call(cons_hp_srv);
		
		//Call the oracle to know the solution (not accessible)
		test_hp_client.call(oracle_ID);
		
		//Compare the consistent hypotheses IDs with the solution ID
		for(int i = 0; i<6;i++){
			printf("Maybe the solution is ID: %d?\n",i);
			if((cons_hp_srv.response.consistent_IDs[i] == true) && ( i == oracle_ID.response.ID)){
				std::cout << "Yes Sherlock you are the best!!" << std::endl;
				std::cout <<"You have found the solution!!"<<std::endl;
				solution_ID = i;
				break;
			}
			else{
				printf("No the solution is not ID: %d!\n",i);
			}
		}
		
		//Solution not found:
		if (solution_ID == -1){
			std::cout << "I' m sorry but you have not found the solution!" << std::endl;
			std::cout<<"You have to collect other hypotheses and come back to me!!" << std::endl;
		}
		//Solution found:
		else{
			//Stop the replanning since the game is end
			say_hp_ID.data = solution_ID;
			solution_pub.publish(say_hp_ID);
			printf("The solution is ID:%d!",solution_ID);
		}
		ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
	}
}

//Main function of the test_hypothesis action 
int main(int argc, char **argv) {
ros::init(argc, argv, "my_rosplan_action_test_hypothesis", ros::init_options::AnonymousName);
ros::NodeHandle nh("~");
cons_hp_client = nh.serviceClient<erl2::ConsistentHypothesis>("/test_consistency");
test_hp_client = nh.serviceClient<erl2::Oracle>("/oracle_solution");
solution_pub = nh.advertise<std_msgs::Int32>("/says_hp", 1000);
KCL_rosplan::TestHypothesisActionInterface my_aci(nh);
my_aci.runActionInterface();
return 0;
}
