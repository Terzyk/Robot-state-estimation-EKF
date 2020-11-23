#!/usr/bin/env python
import rospy as rp
import math
from t_g_filtrs.msg import CarControl
f = 100
msg = CarControl()
t_0 = 0.0
init_scan = 0
t=0.0
def talker():
    global t_0,t,init_scan
    #if init_scan == 0:
        #t_0 = rp.get_rostime()
    #t = rp.get_rostime()
    init_scan =  1
    #msg.u_1 = 0.22*math.sin(float(t.secs-t_0.secs))
    #msg.u_2 = 0.15*math.tanh(float(t.secs-t_0.secs))
    msg.u_1 = 0.22*math.sin(t)
    msg.u_2 = 0.15*math.tanh(t)
    rp.loginfo(msg)
    pub.publish(msg)
    r.sleep()
    t+=1.0/f

if __name__ == '__main__':
    pub = rp.Publisher('CarControl_signals', CarControl)
    rp.init_node('sterownik', anonymous=True)
    r = rp.Rate(f) #10hz
    while not rp.is_shutdown():
        talker()