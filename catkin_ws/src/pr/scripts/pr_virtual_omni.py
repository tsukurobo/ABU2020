#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from pr.msg import RawPower, RawEncoder

class VirtualOmni:
    def __init__(self):
        self.enc_data = RawEncoder()
        self.raw_encoder_pub = \
                rospy.Publisher('raw_encoder', RawEncoder, queue_size = 10)
        self.raw_power_sub = \
                rospy.Subscriber('raw_power', RawPower, self.on_receive_power)
        frequency = float(rospy.get_param('/const/spec/frequency'))
        self.rate = rospy.Rate(frequency)

    def on_receive_power(self, power_msg):
        self.enc_data.e1 = -power_msg.p1
        self.enc_data.e2 = -power_msg.p2
        self.enc_data.e3 = -power_msg.p3

    def start(self):
        while not rospy.is_shutdown():
            self.raw_encoder_pub.publish(self.enc_data)
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('virtual_omni')
    vo = VirtualOmni()
    vo.start()

