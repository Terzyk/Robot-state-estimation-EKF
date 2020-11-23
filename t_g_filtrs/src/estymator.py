#!/usr/bin/env python
import rospy
from CarControl.msg import CarControl

def talker():
    pub = rospy.Publisher('CarControl msg', CarControl)
    rospy.init_node('custom_talker', anonymous=True)
    r = rospy.Rate(10) #10hz
    msg = CarControl()
    msg.u_1 = 1.0
    msg.u_1 = 2.0

    while not rospy.is_shutdown():
        rospy.loginfo(msg)
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass