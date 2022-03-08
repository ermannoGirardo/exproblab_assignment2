#!/usr/bin/env python

## @package erl2
#   \file perceive_hints.py
#   \brief collect hints once the Marker is reached with e.e
#   \author Ermanno Girardo
#   \version 1.0
#
#
#   Subscribes to: <BR>
#        /oracle_hint
#
#
# Description:    
# 
# This node is demanded to perceive the hints generated by the oracle (simulation.cpp) node on the topic /oracle_hint


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

# Global Variables#####
		     ##
hypotheses=[]        ##
		     ##
		     ##
#######################

class Hypothesis:
    """ This is the class that implement an hypothesis """
    
    def __init__(self):
        """ Initialization of the class."""
        self.killer = []
        self.killer_weapon = []
        self.killer_location = []
        self.ID=-1


def perceive_callback(hint):
    """Callback executed when a hint is perceived  """
    #Boolean to keep in mind if a hint is duplicated
    is_double=False
    j=0
    #Check if the hint value is valid
    if(hint.value!='-1'):
    	#Check the type of the hint
        if(hint.key=='who'):
           #Run on all the hypothesis
            while ((j<len(hypotheses[hint.ID].killer))and (is_double==False)):
                #Check if the hint is already perceived
                if(hint.value==hypotheses[hint.ID].murderer[j]):
                    #If the hint is already perceived remove it
                    hypotheses[hint.ID].killer.remove(hint.value)
                    #The hint is already perceived
                    is_double=True
                #update the counter
                j = j + 1
            #Append the new hint
            hypotheses[hint.ID].killer.append(hint.value)

        is_double=False
        j=0   
        #Check the type of the hint
        if(hint.key=='where'):
            #Run on all the hypothesis
            while ((j<len(hypotheses[hint.ID].killer_location))and (is_double==False)):
                #Check if the hint is already perceived
                if(hint.value==hypotheses[hint.ID].killer_location[j]):
                    hypotheses[hint.ID].killer_location.remove(hint.value)
                    #The hint is already perceived
                    is_double=True
                j = j + 1
            hypotheses[hint.ID].killer_location.append(hint.value)

        is_double=False
        j=0     
        #Check the type of the hint
        if(hint.key=='what'):
            #Run on all the hypothesis
            while ((j<len(hypotheses[hint.ID].killer_weapon))and (is_double==False)):
                #Check if the hint is already perceived
                if(hint.value==hypotheses[hint.ID].killer_weapon[j]):
                    hypotheses[hint.ID].killer_weapon.remove(hint.value)
                    #The hint is already perceived
                    is_double=True
                j = j + 1
            hypotheses[hint.ID].killer_weapon.append(hint.value)

    print("New hint is perceived")
    print('ID:',hint.ID,", key: ",hint.key,", value: ",hint.value,"\n")
    






def main():
    """main of the perceive_hints node"""
    rospy.init_node('perceive_hints',anonymous=True)
    rospy.Subscriber("oracle_hint", ErlOracle, perceive_callback)
    #Generate the six possibles hypotheses
    for i in range(6):
        HP = Hypothesis()
        HP.ID=i
        hypotheses.append(HP)
    
    rospy.spin()
    
if __name__ == '__main__':
    main()