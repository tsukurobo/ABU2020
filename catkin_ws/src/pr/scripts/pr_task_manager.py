#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Joy

cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)

def on_receive_joy(joy):
    state = rospy.get_param('/state/control_mode')
    if state == 'manual':
        linear = Vector3(joy.axes[1], -joy.axes[0], 0)
        angular = Vector3(0, 0, -joy.axes[2])
        cmd_vel = Twist(linear, angular)
        cmd_vel_pub.publish(cmd_vel)

def task_manager():
    rospy.init_node('task_manager')

    rospy.Subscriber('joy', Joy, on_receive_joy)

    frequency = float(rospy.get_param('/const/spec/frequency'))
    rate = rospy.Rate(frequency)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        task_manager()
    except rospy.ROSInterruptException:
        pass

