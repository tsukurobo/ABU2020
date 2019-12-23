#!/usr/bin/env python
# -*- coding: utf-8 -*-



import rospy

import tf

from geometry_msgs.msg import Twist

from pr.msg import RawPower, RawEncoder



import numpy as np

import numpy.linalg as LA



# (x,y,ω)に対する回転行列

def R(theta):

    return [[np.cos(theta), -np.sin(theta), 0],

            [np.sin(theta), np.cos(theta), 0],

            [0, 0, 1]]



class ArduinoDriver:

    def __init__(self):

        self.center_to_wheel = float(rospy.get_param('/const/spec/center_to_wheel'))

        self.wheel_radius = float(rospy.get_param('/const/spec/wheel_radius'))

        self.wheel_circumference = 2*np.pi*self.wheel_radius

        self.encoder_resolution = float(rospy.get_param('/const/spec/encoder_resolution'))



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

        self.velocity = [0, 0, 0] # 単位が m/s でないことに注意



        self.raw_power_pub = rospy.Publisher('raw_power', RawPower, queue_size = 10)

        rospy.Subscriber('raw_encoder', RawEncoder, self.on_receive_delta_enc)

        rospy.Subscriber('cmd_vel', Twist, self.on_receive_cmd_vel)



    # rename on_receive_data ?

    def on_receive_delta_enc(self, delta_enc):

        delta_enc = [delta_enc.e1, delta_enc.e2, delta_enc.e3]

        delta_position = LA.multi_dot([R(self.position[2]), self.enc2local_matrix, delta_enc])

        self.position += delta_position



    def on_receive_cmd_vel(self, cmd_vel):

        self.velocity = [cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z]



    def calc_power(self, direction):

        power = LA.multi_dot([self.local2wheel_matrix, R(-self.position[2]), direction])

        maxp  = np.abs(np.amax(power))

        if maxp > 1: power *= 1./maxp

        return power



    def start(self):

        rospy.init_node('arduino_driver')



        frequency = float(rospy.get_param('/const/spec/frequency'))

        self.rate = rospy.Rate(frequency)



        while not rospy.is_shutdown():

            power = self.calc_power(self.velocity)



            data = RawPower()

            pmax = np.amax(map(abs, power))

            if pmax > 1.0: power /= pmax

            data.p1 = int(power[0]*50)

            data.p2 = int(power[1]*50)

            data.p3 = int(power[2]*50)



            self.raw_power_pub.publish(data)

            br = tf.TransformBroadcaster()

            br.sendTransform((self.position[0], self.position[1], 0),

                tf.transformations.quaternion_from_euler(0, 0, self.position[2]),

                rospy.Time.now(), 'base_link', 'odom')

            br.sendTransform((0.09, 0.0, 0.0), tf.transformations.quaternion_from_euler(0,0,0), rospy.Time.now(),\
            "laser", "base_link") #ロボットから見たURGの相対位置のtfをブロードキャスト

            self.rate.sleep()



if __name__ == '__main__':

    driver = ArduinoDriver()

    driver.start()
