# pr_pass.py

import rospy
from std_msgs import Int32
from sensor_msgs import Joy

from pr_msg.msg import PrMsg

buf = PrMsg()
bufpp = Int32
bufkick = Int32
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
    global isfire

    r = rospy.Rate(10)
    pub = rospy.Publisher('pr_main_order', PrMsg, queue_size=None)
    sub = rospy.Subscriber('pr_main_order', PrMsg, cbf)
    subjoy = rospy.Subscriber('joy', Joy, cbjoy)
    pubpp = rospy.Publisher('pp_order', Int32, queue_size=None)
    subpp = rospy.Subscriber('pp_order', Int32, cbpp)
    pubkick = rospy.Publisher('pk_order', Int32, queue_size=None)
    subkick = rospy.Subscriber('pk_order', Int32, cbkick)

    rospy.init_node('pr_pass')

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
            bufpp.data = 1
            pubpp.publish(bufpp)
            while bufpp.data[1] == 1:
                r.sleep()

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
        if buf.kick_ball == 1 and isfire == 1:
            #kick the ball
            bufkick.data = 1
            pubkick.publish(bufkick)
            while bufkick == 1:
                r.sleep()

            #finish
            buf.kick_ball = 0
            pub.publish(buf)
        

        r.sleep()
        
            
        
if __name__ == '__main__':
    try:
        _main()
    except rospy.ROSInterruptException: pass