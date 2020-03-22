#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
#from pr_Msg.msg import *****

twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
goal_pub = rospy.Publisher('goal', Vector3, queue_size = 10)

def on_receive_joy(joy):
    linear = Vector3(-joy.axes[0], joy.axes[1], 0)
    angular = Vector3(0, 0, joy.axes[3])
    twist = Twist(linear, angular)
    twist_pub.publish(twist)

def on_receive_goal(goal):
    goalpos = Vector3()
    goalpos.x = goal.pose.position.x
    goalpos.y = goal.pose.position.y
    rpy = tf.transformations.euler_from_quaternion([goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w])
    goalpos.z = rpy[2]

    goal_pub.publish(goalpos)

#def on_receive_pr_msg(_msg):
#    goalpos = Vector3()
#    if  _msg.**** == 1 and _msg.**** ==  ****:
#        goalpos.x = 
#        goalpos.y = 
#        goalpos.z = 
#        goal_pub.publish(goalpos)
#    elif _msg.**** == 1 and _msg.**** ==  ****:
#        goalpos.x = 
#        goalpos.y = 
#        goalpos.z = 
#        goal_pub.publish(goalpos)
#    elif _msg.**** == 1 and _msg.**** ==  ****:
#        goalpos.x = 
#        goalpos.y = 
#        goalpos.z = 
#        goal_pub.publish(goalpos)
#    elif _msg.**** == 1 and _msg.**** ==  ****:
#        goalpos.x = 
#        goalpos.y = 
#        goalpos.z = 
#        goal_pub.publish(goalpos)

def task_manager():
    rospy.init_node('task_manager')
    state = rospy.get_param('/state/control_mode')
    if state == "manual" or state == "semi-auto" or state == "semi-auto2":
        rospy.Subscriber('joy', Joy, on_receive_joy)
    if state == "semi-auto":
        rospy.Subscriber('move_base_simple/goal', PoseStamped, on_receive_goal)
#    if state == "semi-auto2":
#        rospy.Subscriber('test', , on_receive_pr_msg)

    frequency = float(rospy.get_param('/const/spec/frequency'))
    rate = rospy.Rate(frequency)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        task_manager()
    except rospy.ROSInterruptException:
        pass

