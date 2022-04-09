#!/usr/bin/env python

## @package erl2
#   \file update_knowledge_replan.py
#   \brief Create all the ROSPlan clients and update knowledge base
#   \author Ermanno Girardo
#   \version 1.0
#
#
#   Clients : <BR>
#
#        /rosplan_problem_interface/problem_generation_server
#
#        /rosplan_planner_interface/planning_server
#
#        /rosplan_parsing_interface/parse_plan
#
#        /rosplan_plan_dispatcher/dispatch_plan
#
#        /rosplan_knowledge_base/update
#
#        /rosplan_knowledge_base/clear
#
#    Subscriber : <BR>
#        /says_hp
# 
#           
# Description:    
# 
# This node create all the ROSPlan clients and allow replanning if occurred update the knowledge base
#

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import *
from moveit_commander.conversions import pose_to_list
from erl2.srv import *
from std_srvs.srv import *
import time
import rosplan_dispatch_msgs.srv 
import rosplan_knowledge_msgs.srv
import diagnostic_msgs.msg

#GLOBAL VARIABLES
update_knowledge_client = None
true_hp_ID = -1  #Initialized as not found (-1)



#FUNCTIONS FOR UPDATE THE KNOWLEDGE BASE 

def update_predicate(is_negative,attribute_name,key,value):
    '''Updates the predicate into the knowledge base'''
    update_request=rosplan_knowledge_msgs.srv.KnowledgeUpdateServiceRequest()
    update_request.knowledge.is_negative=is_negative
    update_request.update_type=0
    update_request.knowledge.knowledge_type=1
    update_request.knowledge.attribute_name=attribute_name
    key_value=diagnostic_msgs.msg.KeyValue()
    key_value.key=key
    key_value.value=value
    update_request.knowledge.values.append(key_value)
    global update_knowledge_client
    update_result=update_knowledge_client(update_request)
    if update_result:
    	print('Predicate: ' + attribute_name + 'update success') 
    
def update_instance(instance_name,instance_type):
    '''Add an instance into the knowledge base'''
    update_request=rosplan_knowledge_msgs.srv.KnowledgeUpdateServiceRequest()
    update_request.update_type=0
    update_request.knowledge.knowledge_type=0
    update_request.knowledge.instance_name=instance_name
    update_request.knowledge.instance_type=instance_type
    global update_knowledge_client
    update_result=update_knowledge_client(update_request)
    if update_result:
    	print('Instance: ' + instance_name + 'update success') 


def update_goal(attribute_name,key,value,is_negative):
    '''Add a goal into the knowledge base'''
    update_request=rosplan_knowledge_msgs.srv.KnowledgeUpdateServiceRequest()
    update_request.knowledge.is_negative=is_negative
    update_request.update_type=1
    update_request.knowledge.knowledge_type=1
    update_request.knowledge.attribute_name=attribute_name
    key_value=diagnostic_msgs.msg.KeyValue()
    key_value.key=key
    key_value.value=value
    update_request.knowledge.values.append(key_value)
    global update_knowledge_client
    update_result=update_knowledge_client(update_request)
    if update_result:
    	print('Goal: ' + attribute_name + 'update success')


def says_hp_callback(solution_id):
   '''Receive the ID of the final correct hypothesis'''
   global true_hp_id
   true_hp_id = solution_id

def main():
    """main of the update_knowledge_replan  node, start the game and force replanning until the solution has been found  """
    global update_knowledge_client
    rospy.init_node('update_knowledge_replan',anonymous=True)
    # Wait for all rosplan services
    rospy.wait_for_service('rosplan_problem_interface/problem_generation_server')
    rospy.wait_for_service('rosplan_planner_interface/planning_server')
    rospy.wait_for_service('rosplan_parsing_interface/parse_plan')
    rospy.wait_for_service('rosplan_plan_dispatcher/dispatch_plan')
    rospy.wait_for_service('rosplan_knowledge_base/update')
    rospy.wait_for_service('rosplan_knowledge_base/clear')
    print('All services ready')
    
    # Generates all ROSPlan clients
    prob_gen_client=rospy.ServiceProxy('rosplan_problem_interface/problem_generation_server',Empty)  
    plan_client=rospy.ServiceProxy('rosplan_planner_interface/planning_server',Empty) 
    parse_client=rospy.ServiceProxy('rosplan_parsing_interface/parse_plan',Empty)
    dipatch_client=rospy.ServiceProxy('rosplan_plan_dispatcher/dispatch_plan',rosplan_dispatch_msgs.srv.DispatchService) 
    update_knowledge_client=rospy.ServiceProxy('rosplan_knowledge_base/update',rosplan_knowledge_msgs.srv.KnowledgeUpdateService) 
    clear_knowledge_client=rospy.ServiceProxy('rosplan_knowledge_base/clear',Empty) 
    
    # Declare subscriber for share the final solution ID.
    # Once the final hp has been found the game ends and replan is no more need
    rospy.Subscriber("/says_hp", Int32, says_hp_callback)
    
    
    #Force replanning until solution is found:
    while true_hp_ID == -1:
    	#clear the knowledge
    	clear_knowledge_client()
    	#add instances
    	update_instance('wp1','waypoint')
    	update_instance('wp2','waypoint')
    	update_instance('wp3','waypoint')
    	update_instance('wp4','waypoint')
    	#add predicate
    	update_predicate(False,'not_initialized','','')
    	#add goal
    	update_goal('hypothesis_tested','','',False)
    	#Call all the services in order to initialize the system
    	# 1) Problem generation
    	prob_gen_client()
    	print('Problem generation success')
    	# 2) Plan generation
    	plan_client()
    	print('Plan generation success')
    	# 3) Parse the plan
    	parse_client()
    	print('Plan parsing success')
    	# 4) Dispatch the plan
    	response = dipatch_client()
    	if response.goal_achieved:
    		print('Solution of the plan has been found')
    	else:
    		print('Not possible to find a solution of the plan')
    	time.sleep(1)
   	
   	
if __name__ == '__main__':
    main()
   	
   	
   	
   	
   	 		
    		
    		
    		
   

    	
    	
    	
    	
    	
    	
    	
    	
    	
    	
    	
    	
    	
    	
    	
    	
