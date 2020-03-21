#!/usr/bin/env python
import rospy
#from sensor_msgs import Joy

from pr_msg.msg import PrMsg

#definitions and initalization of variables
buf_order = PrMsg()

#callback function of msg
def pr_cb_msg(getmsg):
    #global buf_order
    buf_order = getmsg

# for waiting a job
def waitjob(job):
    global buf_order
    r = rospy.Rate(10)
    while getattr(buf_order, job) == 1:
        r.sleep()
    if getattr(buf_order, job) == 0:
        return 0
    else:
        return 1


#errorhandle function
def errorhandle():
    rospy.signal_shutdown('error')


#main function
def _main():
    rospy.init_node("pr_main")
    
    #definitions of variables
    global buf_order
    step = 0
    r = rospy.Rate(10)

    #definiton of subscriber
    suborder = rospy.Subscriber("pr_main_order", PrMsg, pr_cb_msg)
    #definitions of publisher 
    pub = rospy.Publisher("pr_main_order", PrMsg,queue_size=1)

    #Main loop of PR
    while not rospy.is_shutdown():
        #rospy.loginfo('top of main loop')
        #pick up pass ball and throw them to TR (x4)
        numpb = 0
        while step == 0 and numpb < 4:
            buf_order.pick_ball = 1
            pub.publish(buf_order)
            if waitjob("pick_ball") == 1:
                errorhandle()
                pass
            else:
                buf_order.pass_ball = 1
                pub.publish(buf_order)
                if waitjob("pass_ball") == 1:
                    errorhandle()
                    pass
                else:
                    rospy.loginfo("success to pass the ball")
                    numpb += 1
        step=1

        #load and kick ball (x3)
        numkb = 0
        while step == 1 and numkb < 3:
            #load
            buf_order.load_ball = 1
            pub.publish(buf_order)
            if waitjob("load_ball") == 1:
                errorhandle()
                pass
            else:
                #kick
                buf_order.kick_ball = 1
                pub.publish(buf_order)
                if waitjob("kick_ball") == 1:
                    errorhandle()
                    pass
                else:
                    numkb += 1
                    rospy.loginfo("success to kick the ball")
        step=3

        #EXTRA ORDER
        if step == 3 and buf_order.extra == 1:

            #(extra) load kick ball (x1)
            buf_order.load_ball = 1
            pub.publish(buf_order)
            if waitjob("load_ball") == 1:
                errorhandle()
                pass
            else:
                pass

            #(extra) kick the ball (x1)
            buf_order.kick_ball = 1
            pub.publish(buf_order)
            if waitjob("kick_ball") == 1:
                errorhandle()
                pass
            else:
                rospy.loginfo("success to kick the ball (extra)")
        step=0

        r.sleep()


if __name__ == '__main__':
    try:
        _main()
    except rospy.ROSInterruptException: pass