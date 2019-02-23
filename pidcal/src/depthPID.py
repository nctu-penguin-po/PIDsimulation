#!/usr/bin/env python
# license removed for brevity

from static_phycal import *
import numpy as np
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray 
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
from rospy.numpy_msg import numpy_msg
import time

state_data = 0
depth_data = 0
depthL = [0 for i in range(10)]
depthR = [0.25, 0.5]

Fkp = 0
FincreaseRate = 0.03

def depth_cb(data):
    global depthL, Fkp, FincreaseRate, depth_data, state_data
    depth_data=data.data
    for i in range(len(depthL)-1, 0, -1):
        depthL[i] = depthL[i-1]
    depthL[0] = depth_data
    if (state_data%2) == 0 or state_data == -1:
        pub_data = [0 for i in range(8)]
        pub_data = Float32MultiArray(data = pub_data)
        pub1.publish(pub_data)
        return
    
    if depth_data < depthR[0]:
        Fkp = Fkp-FincreaseRate*3
    elif depth_data > depthR[1]:
        Fkp = Fkp+FincreaseRate
    else:
        sumt = 0
        for i in range(10):
            sumt = sumt+depthL[i]
        sumt = sumt/5
        if depthR[0] < depthR[1]:
            Fkp = Fkp+FincreaseRate
        else:
            Fkp = Fkp-FincreaseRate*0.5
    #test_mat_all = np.matrix([[0], [0], [az], [0], [0], [0]])
    result_F = Taz*Fkp
    pub_data = [result_F[i] for i in range(8)]
    pub_data = Float32MultiArray(data = pub_data)
    pub1.publish(pub_data)

def Kp_cb(data):
    global FincreaseRate
    FincreaseRate = data.data
    
def state_cb(data):
    global state_data, Fkp
    state_data = data.data
    if (state_data%2) == 0:
        Fkp = 0

rospy.init_node('depthPID',anonymous=True)

rospy.Subscriber('depth', Float32, depth_cb)
rospy.Subscriber('/PIDpara/depth', Float32, Kp_cb)
rospy.Subscriber('/state', Int32, state_cb)

pub1 = rospy.Publisher('/force/depth',Float32MultiArray,queue_size=10)

while not rospy.is_shutdown():
    pass


