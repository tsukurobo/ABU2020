#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
from std_msgs.msg import Float32

rospy.init_node("pr_tf_listener")

tf_listener = tf.TransformListener()

yawPub = rospy.Publisher("yaw_base2map", Float32, queue_size = 10)

#rospy.sleep(5)

yaw = Float32()

r = rospy.Rate(50)

#tf_listener.waitForTransform('base_link','map',rospy.Time(0),rospy.Duration(10.0))
while not rospy.is_shutdown():
    #tf_listener.waitForTransform('base_link','map',rospy.Time(0),rospy.Duration(4.0))
    try:
        (trans, rot)=tf_listener.lookupTransform("map","base_link",rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue

    rpy = tf.transformations.euler_from_quaternion([rot[0],rot[1],rot[2],rot[3]])
    yaw.data = rpy[2]

    yawPub.publish(yaw)

    r.sleep()