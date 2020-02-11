#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
from geometry_msgs.msg import Twist
from pr.msg import RawPower, RawEncoder
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32

import numpy as np
import numpy.linalg as LA
import math

# (x,y,ω)に対する回転行列

def R(theta):
    return [[np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta), np.cos(theta), 0],
            [0, 0, 1]]

I = [[1,0,0],[0,1,0],[0,0,1]]

class ArduinoDriver:

    def __init__(self):

        self.center_to_wheel = float(rospy.get_param('/const/spec/center_to_wheel'))
        self.wheel_radius = float(rospy.get_param('/const/spec/wheel_radius'))
        self.wheel_circumference = 2*np.pi*self.wheel_radius
        self.encoder_resolution = float(rospy.get_param('/const/spec/encoder_resolution'))
        self.kf_intvl = float(rospy.get_param('/const/kf/interval'))
        self.kf_rot = float(rospy.get_param('/const/kf/update_rotation'))
        self.a1 = float(rospy.get_param('/const/kf/alpha1'))
        self.a2 = float(rospy.get_param('/const/kf/alpha2'))
        self.sigma_rot = float(rospy.get_param('/const/kf/initial_sigma_rot'))
        self.sigma_gy = float(rospy.get_param('/const/kf/sigma_gyro'))
        self.frequency = float(rospy.get_param('/const/spec/frequency'))
        self.yaw = 0.0
        self.th_gyro = 0.0
        self.time_pre = 0.0; self.time_now = 0.0
        self.use_kf = int(rospy.get_param('mode/use_kf'))
        self.use_pf = int(rospy.get_param('/mode/use_pf'))
        self.gyrocb_count = 0; self.gyrocb_count_pre = 0

        # 機体中心のローカル座標系で見た速度からモーターの出力値に変換する

        self.local2wheel_matrix = [[-1, 0, -1],
                                   [0.5, np.sqrt(3)/2, -1],
                                   [0.5, -np.sqrt(3)/2, -1]]



        # 車輪の速度から機体中心のローカル座標系で見た速度に変換する

        self.enc2local_matrix = (self.wheel_circumference / self.encoder_resolution) * \
                                LA.inv([[-1, 0, -self.center_to_wheel],
                                        [0.5, np.sqrt(3)/2, -self.center_to_wheel],
                                        [0.5, -np.sqrt(3)/2, -self.center_to_wheel]])


        # (x, y, ω)

        self.position = [0, 0, 0]
        self.th_pre = 0.0
        self.velocity = [0, 0, 0] # 単位が m/s でないことに注意


        self.raw_power_pub = rospy.Publisher('raw_power', RawPower, queue_size = 10)
        rospy.Subscriber('raw_encoder', RawEncoder, self.on_receive_delta_enc)
        rospy.Subscriber('cmd_vel', Twist, self.on_receive_cmd_vel)
        rospy.Subscriber('current_pos', Vector3, self.on_receive_yaw)
        rospy.Subscriber("gyro", Float32, self.on_receive_gyro)


    # rename on_receive_data ?

    def on_receive_delta_enc(self, delta_enc):
        delta_enc = [delta_enc.e1, delta_enc.e2, delta_enc.e3]
        delta_position = LA.multi_dot([R(self.position[2]), self.enc2local_matrix, delta_enc])
        #kf_rot_d = 0
        self.position += delta_position

        if self.use_kf == 1: #KFの使用がオンになっているとき、以下を実行
            self.time_now = rospy.Time.now().to_sec()
            if self.time_now - self.time_pre > self.kf_intvl: #kf_intvl秒ごとにKFによる回転角度の補正を実施
                if abs(self.th_gyro - self.th_pre) > self.kf_rot and self.gyrocb_count - self.gyrocb_count_pre != 0: #ロボットのkf_intv秒間での回転角度が閾値以下の時はKFによる補正を実施しない
                    #以下、kfを用いた計算(回転角度に着目して展開した式を用いている)
                    sigma_rot_pre = self.sigma_rot + self.a1*(delta_position[0]*self.frequency)**2 + self.a1*(delta_position[1]*self.frequency)**2\
                                    + self.a2*(delta_position[2]*self.frequency)**2
                    K_gain = sigma_rot_pre/(self.sigma_gy + sigma_rot_pre)
                    th_new = self.position[2] + K_gain*(self.th_gyro - self.position[2])
                    self.sigma_rot = (1 - K_gain)*sigma_rot_pre

                    self.position[2] = th_new
                        #kf_rot_d = th_new
                    #rospy.loginfo("odometry:%f kf:%f",self.position[2],)
                else:
                    rospy.loginfo("KFを用いずに自己位置を計算しています...")

                rospy.loginfo("self.position[2]=%f",self.position[2])

                self.gyrocb_count_pre = self.gyrocb_count
                self.time_pre = self.time_now
                self.th_pre = self.th_gyro


    def on_receive_cmd_vel(self, cmd_vel):
        self.velocity = [cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z]

    def on_receive_yaw(self, th):
        self.yaw = th.z
        #rospy.loginfo(self.yaw)

    def on_receive_gyro(self, gyrot):
        self.th_gyro = gyrot.data*math.pi/180.0 #ジャイロセンサから送られてくる度数法表記の回転角度を弧度法に変換
        self.gyrocb_count += 1 #この変数は、ジャイロセンサからのメッセージが来なくなったことを検出するために使う.

    def calc_power(self, direction):
        if self.use_pf == 1:
        #power = LA.multi_dot([self.local2wheel_matrix, R(-self.position[2]), direction])
            power = LA.multi_dot([self.local2wheel_matrix, R(-self.yaw), direction])
        else:
            power = LA.multi_dot([self.local2wheel_matrix, I, direction])
        #rospy.loginfo(rpy[2])
        maxp  = np.abs(np.amax(power))
        if maxp > 1: power *= 1./maxp

        return power



    def start(self):

        rospy.init_node('arduino_driver')


        self.rate = rospy.Rate(self.frequency)



        while not rospy.is_shutdown():

            br = tf.TransformBroadcaster()
            br.sendTransform((self.position[0], self.position[1], 0),\
                tf.transformations.quaternion_from_euler(0, 0, self.position[2]),\
                rospy.Time.now(), 'base_link', 'odom')

            br.sendTransform((rospy.get_param('/const/urg_x'), rospy.get_param('/const/urg_y'), 0.0),\
                 tf.transformations.quaternion_from_euler(rospy.get_param('/const/urg_roll'),0,rospy.get_param('/const/urg_yaw')), rospy.Time.now(),\
            "laser", "base_link") #ロボットから見たURGの相対位置のtfをブロードキャスト

            power = self.calc_power(self.velocity)



            data = RawPower()
            pmax = np.amax(map(abs, power))
            if pmax > 1.0: power /= pmax

            data.p1 = int(power[0]*100)
            data.p2 = int(power[1]*100)
            data.p3 = int(power[2]*100)

            self.raw_power_pub.publish(data)

            self.rate.sleep()


if __name__ == '__main__':
    driver = ArduinoDriver()
    driver.start()
