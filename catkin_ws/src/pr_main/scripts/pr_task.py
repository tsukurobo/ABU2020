#!/usr/bin/env python
# pr_pass.py

import rospy
from std_msgs.msg import Int32
from sensor_msgs.msg import Joy

#import custom messages
from pr_msg.msg import PrMsg
from pr_msg.msg import PpMsg
from pr_msg.msg import KickMsg
from pr_msg.msg import MoveMsg
from pr_msg.msg import LoadMsg

buf = PrMsg()
bufpp = PpMsg()
bufkick = KickMsg()
bufmove = MoveMsg()
bufload = LoadMsg()
isfire = 0

class numkick: 
    data = 0

#callback functions
def cbf(getmsg):
    global buf
    buf = getmsg

def cbjoy(joybuf):
    global isfire
    isfire = joybuf.buttons[0]

def cbpp(getpp):
    global bufpp
    bufpp = getpp

def cbkick(getkick):
    global bufkick
    bufkick = getkick

def cbmove(getmove):
    global bufmove
    bufmove = getmove

def cbload(getload):
    global bufload
    bufload = getload

def _main():
    global buf
    global bufpp
    global bufkick
    global bufload
    global isfire

    rospy.init_node('pr_task')
    
    r = rospy.Rate(10)

    pub = rospy.Publisher('pr_main_order', PrMsg, queue_size=1)
    sub = rospy.Subscriber('pr_main_order', PrMsg, cbf)
    subjoy = rospy.Subscriber('joy', Joy, cbjoy)
    pubpp = rospy.Publisher('pp_tpc', PpMsg, queue_size=1)
    subpp = rospy.Subscriber('pp_tpc', PpMsg, cbpp)
    pubkick = rospy.Publisher('kick_tpc', KickMsg, queue_size=1)
    subkick = rospy.Subscriber('kick_tpc', KickMsg, cbkick)
    pubmove = rospy.Publisher('move_tpc', MoveMsg,queue_size=1)
    submove = rospy.Subscriber('move_tpc', MoveMsg, cbmove)
    pubload = rospy.Publisher('load_tpc', LoadMsg, queue_size=None)
    subload = rospy.Subscriber('load_tpc', LoadMsg, cbload)

    while not rospy.is_shutdown():
        rospy.loginfo('top of main loop')

        #pick ball///////////////////////////////////////////////////
        if buf.pick_ball == 1:
            #move to pick up point
            bufmove.moveto = 'pick'
            bufmove.flag = 1
            pubmove.publish(bufmove)
            while bufmove.flag == 1:
                r.sleep()
            if bufmove.flag < 0:
                #error
                pass

            #pick up
            bufpp.pick = 1
            pubpp.publish(bufpp)
            while bufpp.pick == 1:
                r.sleep()
            if bufpp.pick < 0:
                #error
                pass

            #move to pass point
            bufmove.moveto = 'pass'
            bufmove.flag = 1
            pubmove.publish(bufmove)
            while bufmove.flag == 1:
                r.sleep()
            if bufmove.flag < 0:
                #error
                pass

            #finish
            buf.pick_ball = 0
            pub.publish(buf)

        #pass ball//////////////////////////////////////////////////
        if buf.pass_ball == 1 and isfire == 1:
            #wait for reload of pass
            while bufpp.pass_ball == 2:
                r.sleep()
            #pass the ball
            bufpp.pass_ball = 1
            pubpp.publish(bufpp)
            while bufpp == 1:
                r.sleep()

            #finish
            buf.pass_ball = 0
            pub.publish(buf)


        #kick ball////////////////////////////////////////////////////
        if buf.kick_ball == 1:
            #move to kick point
            bufmove.moveto = 'kick'
            bufmove.flag = 1
            pubmove.publish(bufmove)
            while bufmove.flag == 1:
                r.sleep()
            if bufmove.flag < 0:
                #error
                pass

            #kick the ball
            while isfire == 0:  #wait for input from joy
                r.sleep()
            bufkick.launch = 1
            pubkick.publish(bufkick)
            while bufkick.launch == 1:
                r.sleep()
            if bufkick.launch < 0:
                #error
                pass
            
            #send finish code here
            buf.kick_ball = 0
            pub.publish(buf)

            #reload anyo
            bufkick.wind = 1
            pubkick.publish(bufkick)
            while bufkick.wind == 1:
                r.sleep()
            if bufkick < 0:
                #error
                pass

            #finish without finish code (already had been sent)


        #load kick ball////////////////////////////////////////////////
        if buf.load_ball == 1:
            #set or reset tee
            if numkick == 0:
                #first time: set tee
                bufload.tee_set = 1
                pubload.publish(bufload)
                while bufload.tee_set == 1:
                    r.sleep()
                if bufload.tee_set < 0:
                    #error
                    pass
                numkick+=1
            if numkick < 3:
                #reset tee
                bufload.tee_wind = 1
                pubload.publish(bufload)
                while bufload.tee_wind == 1:
                    r.sleep()
                if bufload.tee_wind < 0:
                    #error
                    pass
                numkick += 1
            else:
                numkick = 0
            
            #load ball
            bufload.load = 1
            pubload.publish(bufload)
            while bufload.load == 1:
                r.sleep()
            if bufload < 0:
                #error
                pass

            #finish
            buf.load_ball = 0
            pub.publish(buf)

        r.sleep()
        
if __name__ == '__main__':
    try:
        _main()
    except rospy.ROSInterruptException: pass