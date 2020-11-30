#!/usr/bin/env python
import rospy as rp
import math
from t_g_filtrs.msg import CarControl
from t_g_filtrs.msg import CarState
import numpy as np

msg = CarState()
t_0 = 0.0
init_scan = 0

# wagi stale
w_0b = 0.93
w_0th = 0.88
w_0x = 0.98
w_0y = 0.97

beta_pred = 0.0
q0_pred = 0.0
q1_pred = 0.0
q2_pred = 0.0
beta_est = 0.0
q0_est = 0.0
q1_est = 0.0
q2_est = 0.0

u1 = 0.0
u2 = 0.0
v1 = 0.0
v2 = 0.0
L = 1.5

def state_callback(data):
    global t_0
    global init_scan
    if init_scan == 0:
        t_0 = rp.get_rostime()
    t = rp.get_rostime()
    init_scan =  1
    delta_t = float(t.secs - t_0.secs)
    msg.beta,msg.theta,msg.x,msg.y = kinematyka(data.beta,data.theta,data.x,data.y,delta_t,u1,u2)
    state_pub.publish(msg)
    #rp.loginfo(msg)
    r.sleep()

def control_callback(data):
    global u1,u2
    u1 = data.u_1
    u2 = data.u_2

def kinematyka(beta,q0,q1,q2,delta_t,u_1,u_2):
    global L,v1,v2,w_0b,w_0th,w_0x,w_0y,beta_pred,q0_pred,q1_pred,q2_pred,beta_est,q0_est,q1_est,q2_est

    v1 = (u2/L)*math.tan(beta)
    v2 = u2

    beta_pred = beta_est + u_1*delta_t
    beta_est = w_0b*beta_pred + (1-w_0b)*beta

    q0_pred = q0_est + v1*delta_t
    q0_est = w_0th*q0_pred + (1-w_0th)*q0

    q1_pred = q1_est + v2*math.cos(q0)*delta_t
    q1_est = w_0x*q1_pred + (1-w_0x)*q1

    q2_pred = q2_est + v2*math.sin(q0)*delta_t
    q2_est = w_0y*q2_pred + (1-w_0y)*q2
    return beta,q0,q1,q2


def listener():
    rp.spin()

if __name__ == '__main__':
    state_pub = rp.Publisher('CarState_signals_estimated', CarState)
    rp.init_node('sensor', anonymous=True)
    rp.Subscriber("CarState_signals_with_Larm", CarState, state_callback)
    rp.Subscriber("CarControl_signals", CarControl, control_callback)
    r = rp.Rate(0.01) #100hz
    while not rp.is_shutdown():
        listener()
