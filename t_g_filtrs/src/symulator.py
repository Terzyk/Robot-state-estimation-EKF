#!/usr/bin/env python
import rospy as rp
import math
from t_g_filtrs.msg import CarControl
from t_g_filtrs.msg import CarState
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

msg = CarState()
t_0 = 0.0
init_scan = 0

v1 = 0.0
v2 = 0.0
L = 1.5
beta = 0.0
beta_prim = 0.0
q0_prim = 0.0
q1_prim = 0.0
q2_prim = 0.0
q0 = 0.0
q1 = 0.0
q2 = 0.0
odebrano = 0
n= 1
kinematyka_vect = []
path_msg = Path()
path_msg.header.frame_id = 'map'

def ster_callback(data):
    global t_0
    global init_scan
    if init_scan == 0:
        t_0 = rp.get_rostime()
    t = rp.get_rostime()
    init_scan =  1
    delta_t = float(t.secs - t_0.secs)
    msg.beta,msg.theta,msg.x,msg.y = kinematyka(delta_t,data.u_1,data.u_2)
    state_pub.publish(msg)
    rp.loginfo(msg)
    r.sleep()


def kinematyka(delta_t,u1,u2):
    global beta,v1,v2,L,q0_prim,q1_prim,q2_prim,q0,q1,q2,odebrano,n,path_msgs
    beta = beta + delta_t*u1
    #beta = delta_t*u1
    v1 = (u2/L)*math.tan(beta)
    v2 = u2
    q0_prim = v1
    #q0 = delta_t*q0_prim
    q0 = q0 + delta_t*q0_prim
    q1_prim = v2*math.cos(q0)
    q1 = q1 + delta_t*q1_prim
    #q1 = delta_t*q1_prim
    q2_prim = v2*math.sin(q0)
    q2 = q2 + delta_t*q2_prim
    #q2 = delta_t*q2_prim
    if beta > math.pi:
        beta = 0.0
    if beta < -1.0*math.pi:
        beta = 0.0
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
    state_pub = rp.Publisher('CarState_signals', CarState)
    path_pub = rp.Publisher('path_symulator', Path, queue_size=10)
    rp.init_node('symulator', anonymous=True)
    rp.Subscriber("CarControl_signals", CarControl, ster_callback)
    r = rp.Rate(1) #100hz
    while not rp.is_shutdown():
        listener()
