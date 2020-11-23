#!/usr/bin/env python
import rospy as rp
import math
from t_g_filtrs.msg import CarControl
from t_g_filtrs.msg import CarState
import numpy as np

msg = CarState()
t_0 = 0.0
init_scan = 0

w_0b = np.random.rand(0.2)
w_0th = np.random.rand(0.2)
w_0x = np.random.rand(0.2)
w_0y = np.random.rand(0.2)
beta_pred = 0.0
q0_pred = 0.0
q1_pred = 0.0
q2_pred = 0.0


kinematyka_vect = []

def ster_callback(data):
    global t_0
    global init_scan
    if init_scan == 0:
        t_0 = rp.get_rostime()
    t = rp.get_rostime()
    init_scan =  1
    delta_t = float(t.secs - (t_0.secs)*r)
    msg.beta,msg.theta,msg.x,msg.y = kinematyka(data.beta,data.q0,data.q1,data.q2,delta_t)

    print("####")
    state_pub.publish(msg)
    rp.loginfo(msg)
    r.sleep()


def kinematyka(beta,q0,q1,q2,delta_t):
    global w_0b,w_0th,w_0x,w_0y,beta_pred,q0_pred,q1_pred,q2_pred

    beta_pred = beta + delta_t*
    beta = w_0b * beta_pred + (1 - w_0b) * beta
    q0 = w_0th * beta_pred + (1 - w_0th) * beta
    q1 = w_0x * beta_pred + (1 - w_0x) * beta
    q2 = w_0y * beta_pred + (1 - w_0y) * beta
    return beta,q0,q1,q2


def listener():
    rp.spin()

if __name__ == '__main__':
    state_pub = rp.Publisher('CarState_signals', CarState)
    rp.init_node('sensor', anonymous=True)
    rp.Subscriber("CarState_signals", CarControl, ster_callback)
    r = rp.Rate(0.01) #100hz
    while not rp.is_shutdown():
        listener()
