#! /usr/bin/env python

## @package erl2
#   \file go_to_point.py
#   \brief This node implement the go_to_point action
#   \author Ermanno Girardo
#   \version 1.0
#   \date 15/04/2022
#
#   \details
#
#   Subscribes to: <BR>
#        /odom
#
#        /velocities
#
#   Publishes to: <BR>
#        /cmd_vel
#
#   Actions : <BR>
#        /go_to_point
#          
# Description:    
# 
# This node commands our robot to the target via a ROS action, the angular and linear speeds of the robot can be changed by publishing in the topic /velocities (to which this node is subscribed).
#

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
import math
import actionlib
import actionlib.msg
import erl2.msg

# robot state variables
## Contains the current position of the robot (update with a subscriber callback)
position_ = Point()
position_.x = 0
position_.y = 0

## Contains the yaw of the robot
yaw_ = 0

## Contains the current state of the robot
state_ = 0

## Publisher
pub_ = None

# parameters for control
yaw_precision_ = math.pi / 9  # +/- 20 degree allowed
yaw_precision_2_ = math.pi / 90  # +/- 2 degree allowed
dist_precision_ = 0.1
kp_a = -3.0 
kp_d = 0.2
ub_a = 0.6
lb_a = -0.5
ub_d = 0.6

## Linear speed of the robot, can be modified via the callback of the subscriber to /velocities
lin_vel = 1

## Angular speed of the robot, can be modified via the callback of the subscriber to /velocities
ang_vel = 1

## action server
act_s = None

def clbk_odom(msg):
    """Callback of the subscriber at odom, save the robot positions in global variables"""
    global position_
    global yaw_

    # position
    position_ = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]

    
def clbk_vel(msg):
    """Callback of the subscriber at velocities, save the velocities in global variables"""
    global lin_vel,ang_vel
    lin_vel=msg.linear.x
    ang_vel=msg.angular.z


def change_state(state):
    """function for changing the state of the robot"""
    global state_
    state_ = state
    print ('State changed to [%s]' % state_)

def normalize_angle(angle):
    """function for normalize an angle"""
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

def fix_yaw(des_pos):
    """function to direct the robot towards the target (rotation)"""
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)
    #rospy.loginfo(err_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = -5*ang_vel*err_yaw
        if twist_msg.angular.z > ang_vel:
            twist_msg.angular.z = ang_vel
        elif twist_msg.angular.z < -ang_vel:
            twist_msg.angular.z = -ang_vel
    pub_.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(1)

def go_straight_ahead(des_pos):
    """function to move the robot towards the target(translation)"""
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) +
                        pow(des_pos.x - position_.x, 2))
    err_yaw = normalize_angle(desired_yaw - yaw_)
    #rospy.loginfo(err_yaw)

    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = lin_vel
        #if twist_msg.linear.x > ub_d:
        #    twist_msg.linear.x = ub_d

        twist_msg.angular.z = -5*ang_vel*err_yaw
        pub_.publish(twist_msg)
    else: # state change conditions
        #print ('Position error: [%s]' % err_pos)
        change_state(2)

    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(0)

def fix_final_yaw(des_yaw):
    """function to adjust the orientation of the robot(rotation)"""
    err_yaw = normalize_angle(des_yaw - yaw_)
    #rospy.loginfo(err_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = -5*ang_vel*err_yaw
        if twist_msg.angular.z > ang_vel:
            twist_msg.angular.z = ang_vel
        elif twist_msg.angular.z < -ang_vel:
            twist_msg.angular.z = -ang_vel
    pub_.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(3)
        
def done():
    """function that stop the robot"""
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub_.publish(twist_msg)


#action server callback
def go_to_point(goal):
    """This is the callback of the action server, guide the robot to the goal, if the goal is canceled the robot stops"""
    global act_s, position_ ,yaw_ ,pub_
    desired_position = Point()
    desired_position.x = goal.x
    desired_position.y = goal.y
    des_yaw = goal.theta
    change_state(0)
    success = True
    
    feedback = erl2.msg.go_to_pointFeedback()
    result = erl2.msg.go_to_pointResult()

    while not rospy.is_shutdown():
        if act_s.is_preempt_requested():
            rospy.loginfo('Goal cancelled')
            feedback.stat = "Goal cancelled"
            feedback.x = position_.x
            feedback.y = position_.y
            feedback.theta = yaw_
            act_s.publish_feedback(feedback)
            act_s.set_preempted()
            twist_msg = Twist()
            twist_msg.linear.x = 0
            twist_msg.angular.z = 0
            pub_.publish(twist_msg)
            success = False
            break

        elif state_ == 0:
            feedback.stat = "fixing start yaw"
            feedback.x = position_.x
            feedback.y = position_.y
            feedback.theta = yaw_
            act_s.publish_feedback(feedback)
            fix_yaw(desired_position)
        elif state_ == 1:
            feedback.stat = "going straight"
            feedback.x = position_.x
            feedback.y = position_.y
            feedback.theta = yaw_
            act_s.publish_feedback(feedback)
            go_straight_ahead(desired_position)
        elif state_ == 2:
            feedback.stat = "fixing final yaw"
            feedback.x = position_.x
            feedback.y = position_.y
            feedback.theta = yaw_
            act_s.publish_feedback(feedback)
            fix_final_yaw(des_yaw)
        elif state_ == 3:
            feedback.stat = "target reached!"
            feedback.x = position_.x
            feedback.y = position_.y
            feedback.theta = yaw_
            act_s.publish_feedback(feedback) 
            done()     
            break
        
    if success:
        rospy.loginfo('Goal: Succeeded!')
        act_s.set_succeeded(result)    
    
def main():
    """main of the go_to_point node"""
    global pub_,act_s
    rospy.init_node('go_to_point')
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    sub_velocities = rospy.Subscriber('/velocities', Twist, clbk_vel)
    act_s = actionlib.SimpleActionServer('/go_to_point', erl2.msg.go_to_pointAction, go_to_point, auto_start=False) #action server
    act_s.start()
    rospy.spin()
    
if __name__ == '__main__':
    main()
