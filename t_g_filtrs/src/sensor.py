#!/usr/bin/env python
import rospy as rp
import math
from t_g_filtrs.msg import CarControl
from t_g_filtrs.msg import CarState
import numpy as np
import random
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

msg = CarState()
t_0 = 0.0
init_scan = 0

path_msg = Path()
path_msg.header.frame_id = 'map'

odebrano = 0
n= 1

mu,sigma = 0.01,0.01

def ster_callback(data):
    global t_0
    global init_scan
    if init_scan == 0:
        t_0 = rp.get_rostime()
    t = rp.get_rostime()
    init_scan =  1
    delta_t = float(t.secs - t_0.secs)
    msg.beta,msg.theta,msg.x,msg.y = kinematyka(data.beta,data.theta,data.x,data.y,delta_t)
    state_pub.publish(msg)
    #rp.loginfo(msg)
    #r.sleep()


def kinematyka(beta,q0,q1,q2,delta_t):
    global mu,sigma,odebrano,n,path_msgs
    r_beta = np.random.normal(mu, sigma, 1)
    r_q0 = np.random.normal(mu, sigma, 1)
    r_q1 = np.random.normal(mu, sigma, 1)
    r_q2 = np.random.normal(mu, sigma, 1)
    print(r_beta)
    print(r_q0)
    print(r_q1)
    print(r_q2)
    beta = beta + r_beta
    q0 = q0 + r_q0
    q1 = q1 + r_q1
    q2 = q2 + r_q2
    # path #
    pose = PoseStamped()
    pose.pose.position.x = q1
    pose.pose.position.y = q2
    pose.pose.position.z = 0
    pose.pose.orientation.x = 0
    pose.pose.orientation.y = 0
    pose.pose.orientation.z = 0
    pose.pose.orientation.w = 1
    pose.header.frame_id = 'map'
    pose.header.stamp = rp.Time.now()
    path_msg.poses.append(pose)
    odebrano+=1
    if odebrano == n*10:
        path_pub.publish(path_msg)
        n+=1
    return beta,q0,q1,q2


def listener():
    rp.spin()

if __name__ == '__main__':
    path_pub = rp.Publisher('path_sensor', Path, queue_size=10)
    state_pub = rp.Publisher('CarState_signals_with_Larm', CarState)
    rp.init_node('sensor', anonymous=True)
    rp.Subscriber("CarState_signals", CarState, ster_callback)
    r = rp.Rate(0.01) #100hz
    while not rp.is_shutdown():
        listener()
