#!/usr/bin/env python
# -*- coding: utf-8 -*-

#simple motion plannner

import rospy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
import tf
import math


#gpos.z -> theta
class SimpleMotionPlanner:

	def __init__(self):
		self.goal = [0.0, 0.0, 0.0]

	def startNavigation(self,kpv, kdv, kpr, kdr, r, th, max_v, max_r): #navigate the robot from current postion to the goal

		tf_listener = tf.TransformListener() 
		currentpos = [0.0, 0.0, 0.0] #(x, y, theta)
		lastpos = [0.0, 0.0, 0.0]
		#startpos = [0.0, 0.0]
		#lastth = 0.0
		does_reached_goal = 0
		vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size = 10)
		vel_cmd = Twist()
		flag = 0
		delta_th = 0.0 #difference between current rotation and the goal rotation
		rate = rospy.Rate(10)
		print("navigation started")

		while not rospy.is_shutdown(): #simple(st) algorithm using PD control
			try: #get current postion of the robot
				(trans, rot) = tf_listener.lookupTransform("map", "base_link", rospy.Time(0))
				currentpos[0] = trans[0]; currentpos[1] = trans[1]
				rpy = tf.transformations.euler_from_quaternion([rot[0],rot[1],rot[2],rot[3]])
				currentpos[2] = rpy[2]

				if flag == 0: #in the first loop, lastpos is equal to currentpos 
					#lastth = currentpos[2]
					lastpos[0] = currentpos[0]; lastpos[1] = currentpos[1]; lastpos[2] = currentpos[2]
					self.goal[0] = currentpos[0]; self.goal[1] = currentpos[1]; self.goal[2] = currentpos[2]
					#theta = math.atan2(gpos.y - currentpos[1], gpos.x - currentpos[0])
					#last_d = d
					flag += 1

				delta_th = currentpos[2] - self.goal[2]

				if delta_th > math.pi:
					delta_th = -(2*math.pi - currentpos[2] + self.goal[2])
				if delta_th < -math.pi:
					delata_th = 2*math.pi + currentpos[2] - self.goal[2]

				#if (currentpos[0] - gpos.x)**2 + (currentpos[1] - gpos.y)**2 < r**2 and \
				#	abs(delta_th) < th:
				#	break

				#if startpos[0] == gpos.x:
				#	d = currentpos[0] - gpos.x
				#else:
				#	d = (startpos[1] - gpos.y)*(currentpos[0] - startpos[0])/(startpos[0] - gpos.x) - currentpos[1] + startpos[1]

				vel_cmd.linear.x = -kpv*(currentpos[0] - self.goal[0]) - kdv*(currentpos[0] - lastpos[0])#max_v*math.cos(theta) - kpv*d*math.sin(theta) - kdv*(d - last_d) #calculate velocity using PD control
				vel_cmd.linear.y = -kpv*(currentpos[1] - self.goal[1]) - kdv*(currentpos[1] - lastpos[1])#max_v*math.sin(theta) + kpv*d*math.cos(theta) - kdv*(d - last_d)
				vel_cmd.angular.z = -kpr*delta_th - kdr*(currentpos[2] - lastpos[2])

				if vel_cmd.linear.x > max_v: vel_cmd.linear.x = max_v
				if vel_cmd.linear.y > max_v: vel_cmd.linear.y = max_v
				if vel_cmd.angular.z > max_r: vel_cmd.angular.z = max_r
				if vel_cmd.linear.x < -max_v: vel_cmd.linear.x = -max_v
				if vel_cmd.linear.y < -max_v: vel_cmd.linear.y = -max_v
				if vel_cmd.angular.z < -max_r: vel_cmd.angular.z = -max_r


				#last_d = d
				#last_th = currentpos[2]
				lastpos[0] = currentpos[0]; lastpos[1] = currentpos[1]; lastpos[2] = currentpos[2]

			except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				print("couldn't get current position of the robot")

			vel_pub.publish(vel_cmd)

			rate.sleep()

		#vel_cmd.linear.x = 0.0
		#vel_cmd.linear.y = 0.0
		#vel_cmd.angular.z = 0.0
		#vel_pub.publish(vel_cmd)
		#print("navigation ended")

	def setGoal(self,gpos):
		self.goal[0] = gpos.x; self.goal[1] = gpos.y; self.goal[2] = gpos.z



if __name__ == "__main__":
	smp = SimpleMotionPlanner()
	rospy.init_node("simple_motion_planner")
	rospy.Subscriber("goal", Vector3, smp.setGoal)
	lin_kp = rospy.get_param("/const/smp/linear_kp");
	lin_kd = rospy.get_param("/const/smp/linear_kd");
	ang_kp = rospy.get_param("/const/smp/angular_kp");
	max_vel_lin = rospy.get_param("/const/smp/max_lin_vel");
	max_vel_ang = rospy.get_param("/const/smp/max_ang_vel");
	smp.startNavigation(lin_kp, lin_kd, ang_kp, 0.0, 0.10, 0.13, max_vel_lin, max_vel_ang) #左から順に、並進運動用のP、Dゲイン、回転運動用のP、Dゲイン、ナビゲーション終了条件(ゴールとの距離、回転の差の順。今は使用していない)
	rospy.spin()												  #最大並進速度、最大角速度

