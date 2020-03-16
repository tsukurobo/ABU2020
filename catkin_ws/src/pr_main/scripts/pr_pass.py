# pr_pass.py

import rospy
import std_msgs

from pr_msg.msg import PrMsg

def _main():
    pub = rospy.Publisher('pr_main_order', PrMsg, queue_size=None)
    
    rospy.init_node('pr_pass')

    while not rospy.is_shutdown():
        


if __name__ == '__main__':
    try:
        _main()
    except rospy.ROSInterruptException: pass