# pr_pass.py

import rospy
from std_msgs import Int32
from sensor_msgs import Joy

#import custom messages
from pr_msg.msg import PrMsg
from pr_msg.msg import PpMsg
from pr_msg.msg import KickMsg

buf = PrMsg()
bufpp = PpMsg()
bufkick = KickMsg()
isfire = int()


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

# for waiting a job
def waitjob(job):
    global order_buf
    r = rospy.Rate(10)
    while getattr(buf, job) == 1:
        r.sleep()
    if getattr(buf, job) == 0:
        return 0
    else:
        return 1

def _main():
    global buf
    global bufpp
    global bufkick
    global isfire

    r = rospy.Rate(10)

    pub = rospy.Publisher('pr_main_order', PrMsg, queue_size=None)
    sub = rospy.Subscriber('pr_main_order', PrMsg, cbf)
    subjoy = rospy.Subscriber('joy', Joy, cbjoy)
    pubpp = rospy.Publisher('pp_tpc', PpMsg, queue_size=None)
    subpp = rospy.Subscriber('pp_tpc', PpMsg, cbpp)
    pubkick = rospy.Publisher('kick_tpc', KickMsg, queue_size=None)
    subkick = rospy.Subscriber('kick_tpc', KickMsg, cbkick)

    rospy.init_node('pr_task')

    while not rospy.is_shutdown():
        #pick ball
        if buf.pick_ball == 1:
            #move to pick up point
            buf.moveto = 'pick'
            pub.publish(buf)
            if waitjob('moveto') == 1:
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
            buf.moveto = 'pass'
            pub.publish(buf)
            if waitjob('moveto') == 1:
                #error
                pass

            #finish
            buf.pick_ball = 0
            pub.publish(buf)

        #pass ball
        if buf.pass_ball == 1 and isfire == 1:
            #pass the ball
            bufpp.data[2] = 1
            pubpp.publish(bufpp)
            while bufpp == 1:
                r.sleep()

            #finish
            buf.pass_ball = 0
            pub.publish(buf)

        #kick ball
        if buf.kick_ball == 1:
            #move to pick up point
            buf.moveto = 'kick'
            pub.publish(buf)
            if waitjob('moveto') == 1:
                #error
                pass

            #kick the ball
            while isfire == 0:  #wait for input from joy
                pass
            bufkick.launch = 1
            pubkick.publish(bufkick)
            while bufkick.launch == 1:
                r.sleep()
            if bufkick.launch < 0:
                #error
                pass

            #reload anyo
            bufkick.wind = 1
            pubkick.publish(bufkick)
            while bufkick.launch == 1:
                r.sleep()
            if bufkick < 0:
                #error
                pass

            #finish
            buf.kick_ball = 0
            pub.publish(buf)
        
        #loading kick ball
        if buf.load_ball == 1:
            pass

        r.sleep()
        
if __name__ == '__main__':
    try:
        _main()
    except rospy.ROSInterruptException: pass