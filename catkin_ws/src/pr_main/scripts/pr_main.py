import rospy
import std_msgs
#from sensor_msgs import Joy

from pr_msg.msg import PrMsg

#definitions and initalization of variables
order_buf = PrMsg()
dict_order = vars(order_buf)
#dict_order = {}
#for key in order_buf.__dict__.keys():
#    dict_order[key] = 0

#definitions of pubkisher 
pub = rospy.Publisher("pr_main_order", PrMsg, queue_size=None)

#callback function of msg
def pr_cb_msg(getmsg):
    pass

# for waiting a job
def waitjob(job):
    r = rospy.Rate(10)
    while getattr(order_buf, job) == 1:
        r.sleep()
    if getattr(order_buf, job) == 2:
        return 0
    else:
        return 1


#main function
def _main():
    rospy.init_node("pr_main")
    
    #definitions of variables
    global order_buf
    step = 0
    r = rospy.Rate(10)

    #definiton of subscriber
    subOrder = rospy.Subscriber("pr_main_order", PrMsg, pr_cb_msg)

    #Main loop of PR
    while not rospy.is_shutdown():
        #pick up pass ball and throw them to TR (x4)
        numpb = 0
        while numpb < 4:
            order_buf.pick_ball = 1
            pub.publish(order_buf)
            if waitjob("pick_ball") == 1:
                #error
                pass
            else:
                order_buf.pass_ball = 1
                pub.publish(order_buf)
                if waitjob("pass_ball") == 1:
                    #error
                    pass
                else:
                    rospy.loginfo("success to pass the ball")
                    numpb += 1
        step=1

        #load kick ball (x3)
        numkb = 0
        if step == 1: while numkb < 3:
            order_buf.load_ball = 1
            pub.publish(order_buf)
            if waitjob("load_ball") == 1:
                #error
                pass
            else:
                numkb += 1
        step=2

        #kick the ball (x3)
        if step == 2: while numkb > 0:
            order_buf.kick_ball = 1
            pub.publish(order_buf)
            if waitjob("kick_ball") == 1:
                #error
                pass
            else:
                rospy.loginfo("success to kick the ball")
                numkb -= 1
        step=3

        #EXTRA ORDER
        if step == 3 and order_buf.extra == 1:

            #(extra) load kick ball (x1)
            order_buf.load_ball = 1
            pub.publish(order_buf)
            if waitjob("load_ball") == 1:
                #error
                pass
            else:
                pass

            #(extra) kick the ball (x1)
            order_buf.kick_ball = 1
            pub.publish(order_buf)
            if waitjob("kick_ball") == 1:
                #error
                pass
            else:
                rospy.loginfo("success to kick the ball (extra)")
        step=0

        r.sleep()


if __name__ == '__main__':
    try:
        _main()
    except rospy.ROSInterruptException: pass