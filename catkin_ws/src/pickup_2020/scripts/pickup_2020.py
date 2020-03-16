#!/usr/bin/env python

import rospy
#from std_msgs.msg import String

import std_msgs

#joy
from sensor_msgs.msg import Joy


from pr_msg.msg import PrMsg

pub = rospy.Publisher('command_serial', PrMsg, queue_size=10)

buttons = PrMsg()
slide_pwm = 0
lift_pwm = 0
turn_pwm = 0


def callback_joy(buf):

    buttons.pick_lift = lift_pwm * buf.axes[5]
    buttons.pick_grasp = buf.buttons[2] - buf.buttons[0]
    buttons.pick_slide = slide_pwm * buf.axes[4]
    buttons.pick_turn = turn_pwm * (buf.buttons[6]-buf.buttons[7])
    buttons.pass_tee = buf.buttons[3] - buf.buttons[1]
    buttons.kick_roll = buf.buttons[5]
    buttons.kick_fire = buf.buttons[9]
    
    rospy.loginfo(lift_pwm)
    pub.publish(buttons)



def ma_in():
    rospy.init_node('pickup_node', anonymous=True)

    global lift_pwm
    global turn_pwm
    global slide_pwm

    slide_pwm = int (rospy.get_param("/pick_task_manager/slide_pwm"))
    turn_pwm = int (rospy.get_param("/pick_task_manager/turn_pwm"))
    lift_pwm = int (rospy.get_param("/pick_task_manager/lift_pwm"))
    sub = rospy.Subscriber('joy', Joy, callback_joy)
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        rospy.loginfo(buttons)
        #rospy.loginfo(slide_pwm)
        r.sleep()Kiss

if __name__ == '__main__':
    try:
        ma_in()
    except rospy.ROSInterruptException: pass
