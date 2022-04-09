#!/usr/bin/env python

## @package exproblab_ass2
#   \file replan.py
#   \brief Cancel the current plan in order to force replan
#   \author Ermanno Girardo
#   \version 1.0
#
#
#   Services: <BR>
#        /replan_service
#          
# Description:    
# 
# This node provide a service to cancel the current plan and force a replan trough the node update_knowledge_replan
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
from erl2.msg import *
from std_srvs.srv import *
import time
import rosplan_dispatch_msgs.srv 
import array
import numpy as np

def replan_callback(req):
    '''Callback of the replan service. It is demanded to cancel plan in order create a new plan to perceive new hypothesis'''
    #See if the service is ready
    rospy.wait_for_service('/rosplan_plan_dispatcher/cancel_dispatch')
    try:
    	cancel_plan_client = rospy.ServiceProxy('/rosplan_plan_dispatcher/cancel_dispatch', Empty)
    	success = cancel_plan_client()
    	if success:
    		print("Plan canceled successfully")
    except rospy.ServiceException as error:
    	print("Client was not able to call the service: %s" %error)
    	
    return True




def main():
    """main of the replan node"""
    rospy.init_node('replan',anonymous=True)
    replan_service = rospy.Service('replan_service', Replan, replan_callback)
    print("replanner online")
    rospy.spin()
    
if __name__ == '__main__':
    main()
