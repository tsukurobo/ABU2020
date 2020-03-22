#!/usr/bin/env python
# -*- coding: utf-8 -*-

import serial
import rospy
from std_msgs.msg import Float32

##############gyro_node##############

if __name__ == "__main__":
    rospy.init_node("gyro_node")
    port = rospy.get_param("~port")
    
    with serial.Serial(port, 57600) as gyro_uno:
        gyro_pub = rospy.Publisher("gyro", Float32, queue_size = 1000)
        data_gy = Float32()
        #loop_r = rospy.Rate()

        print("gyro_node: opened serial port: %s" % port)

        while not rospy.is_shutdown():
            data_gy_str = gyro_uno.readline()
            try:
                data_gy.data = float(data_gy_str)
                gyro_pub.publish(data_gy)
            except ValueError:
                pass



        
