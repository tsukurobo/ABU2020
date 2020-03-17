# pr_pass.py

import rospy
from std_msgs import Int32
from sensor_msgs import Joy

from pr_msg.msg import PrMsg

buf = PrMsg()
bufpick = Int32(0)
bufpass = Int32(0)

def cbf(getmsg):
    global buf
    buf = getmsg

def cbjoy(joybuf):
    global isfire
    isfire = joybuf.buttons[0]

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

    r = rospy.Rate(10)
    pub = rospy.Publisher('pr_main_order', PrMsg, queue_size=None)
    sub = rospy.Subscriber('pr_main_order', PrMsg, cbf)
    subjoy = rospy.Subscriber('joy', Joy, cbjoy)
    pubpick = rospy.Publisher('pp_order', Int32, queue_size=None)

    rospy.init_node('pr_pass')

    while not rospy.is_shutdown():
        if buf.pass_ball == 1:
            #move to pick up point
            buf.moveto = 'pick'
            pub.publish(buf)
            if waitjob('moveto') == 1:
                #error
                pass

            #pick up
            bufpick.data = 1
            pubpick.publish(bufpick)
            while buf.data == 1:
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
            bufpass.data = 1
            pub.publish(bufpass)
            while bufpass == 1:
                pass

            #finish
            buf.pass_ball = 0
            pub.publish(buf)
            
        
if __name__ == '__main__':
    try:
        _main()
    except rospy.ROSInterruptException: pass