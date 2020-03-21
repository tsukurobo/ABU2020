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

buf = PrMsg()
bufpp = PpMsg()
bufkick = KickMsg()
bufmove = MoveMsg()
bufload = 0
isfire = 0  #trigger of pass ball or kick ball
startkick = 0 #trigger of biginning of kick phase 

#publishers
pub = rospy.Publisher('pr_main_order', PrMsg, queue_size=1)
pubpp = rospy.Publisher('pp_tpc', PpMsg, queue_size=1)
pubkick = rospy.Publisher('kick_tpc', KickMsg, queue_size=1)
pubmove = rospy.Publisher('move_tpc', MoveMsg,queue_size=1)
pubload = rospy.Publisher('load_tpc', Int32, queue_size=None)

#classes as static valuable
class numkick: 
    data = 0

class numkick:
    data = 0

class numload:
    data = 1

#callback functions
def cbf(getmsg):
    global buf
    buf = getmsg

def cbjoy(joybuf):
    global isfire
    #set keybind of joy
    isfire = joybuf.buttons[0]
    startkick - joybuf.buttons[1]

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

#error handle function
#send -1
def errorhandle():
    global pub
    global pubpp
    global pubkick
    global pubmove
    global pubload
    buf = PrMsg()
    buf.kick_ball = -1
    buf.load_ball = -1
    buf.pass_ball = -1
    buf.pick_ball = -1
    bufpp = PpMsg()
    bufpp.pick = -1
    bufpp.launch = -1
    bufkick = KickMsg()
    bufkick.wind = -1
    bufkick.launch = -1
    bufload = -1

    pub.publish(buf)
    pubpp.publish(bufpp)
    pubkick.publish(bufkick)
    pubmove.publish(bufmove)
    pubload.publish(bufload)

#main funcion
def _main():
    global buf
    global bufpp
    global bufkick
    global bufload
    global isfire
    global startkick

    rospy.init_node('pr_task')
    
    r = rospy.Rate(10)

    global pub
    sub = rospy.Subscriber('pr_main_order', PrMsg, cbf)
    subjoy = rospy.Subscriber('joy', Joy, cbjoy)
    global pubpp
    subpp = rospy.Subscriber('pp_tpc', PpMsg, cbpp)
    global pubkick
    subkick = rospy.Subscriber('kick_tpc', KickMsg, cbkick)
    global pubmove
    submove = rospy.Subscriber('move_tpc', MoveMsg, cbmove)
    global pubload
    subload = rospy.Subscriber('load_tpc', Int32, cbload)

    while not rospy.is_shutdown():
        rospy.loginfo('top of main loop')

        #pick ball///////////////////////////////////////////////////
        if buf.pick_ball == 1:
            #move to pick up point
            bufmove.moveto = 'pick'+str(numpick)
            if numkick < 5:
                numkick+=1
            else:
                numkick=0
            bufmove.flag = 1
            pubmove.publish(bufmove)
            while bufmove.flag == 1:
                r.sleep()
            if bufmove.flag < 0:
                errorhandle()
                pass

            #pick up
            bufpp.pick = 1
            pubpp.publish(bufpp)
            while bufpp.pick == 1:
                r.sleep()
            if bufpp.pick < 0:
                errorhandle()
                pass

            #move to pass point
            bufmove.moveto = 'pass'
            bufmove.flag = 1
            pubmove.publish(bufmove)
            while bufmove.flag == 1:
                r.sleep()
            if bufmove.flag < 0:
                errorhandle()
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
                errorhandle()
                pass

            #kick the ball
            while isfire == 0:  #wait for input from joy
                r.sleep()
            bufkick.launch = 1
            pubkick.publish(bufkick)
            while bufkick.launch == 1:
                r.sleep()
            if bufkick.launch < 0:
                errorhandle()
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
                errorhandle()
                pass

            #finish without finish code (already had been sent)


        #load kick ball////////////////////////////////////////////////
        if buf.load_ball == 1:
            #load ball
            bufload = 1 + numload
            if numload == 0:
                #first time, wait for loading to auto loading system by human
                pubload.publish(2)
                while bufload == 2:
                    r.sleep()
                    #when human send message of end of loading through joycon, go to next step
                    if startkick == 1:
                        break
                if bufload < 0:
                    errorhandle()
                    pass 
            if numload < 3:
                numload += 1
            else:
                numload = 0
            pubload.publish(bufload)
            while bufload != 6:
                r.sleep()
            if bufload < 0:
                errorhandle()
                pass

            #finish
            buf.load_ball = 0
            pub.publish(buf)

        r.sleep()
        
if __name__ == '__main__':
    try:
        _main()
    except rospy.ROSInterruptException: pass