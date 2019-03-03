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
depthR = [0.3, 0.35, 0.6]

Fkp = 0
FincreaseRate = 0.05

def depth_cb(data):
    global depthL, Fkp, FincreaseRate, depth_data, state_data
    depth_data=data.data
    for i in range(len(depthL)-1, 0, -1):
        depthL[i] = depthL[i-1]
    depthL[0] = depth_data-depthR[1]
    if (state_data%2) == 0 or state_data == -1:
        pub_data = [0 for i in range(8)]
        pub_data = Float32MultiArray(data = pub_data)
        pub1.publish(pub_data)
        return
    
    D = depthL[0] - depthL[2]
    E = depth_data-depthR[1]
    Fkp = Fkp+FincreaseRate*E+FincreaseRate*D*1
    '''
    if depth_data < depthR[0]:
        Fkp = Fkp-FincreaseRate*5
    elif depth_data > depthR[2]:
        Fkp = Fkp+FincreaseRate*5
    elif depth_data > depthR[1]:
        Fkp = Fkp+FincreaseRate
    else:
        if depthL[2] < depthL[0]:
            Fkp = Fkp+FincreaseRate*3
        elif depthL[2] > depthL[0]:
            Fkp = Fkp-FincreaseRate*10
    '''
    #test_mat_all = np.matrix([[0], [0], [az], [0], [0], [0]])
    result_F = Taz*Fkp
    pub_data = [result_F[i] for i in range(8)]
    pub_data = Float32MultiArray(data = pub_data)
    pub1.publish(pub_data)

def Kp_cb(data):
    global FincreaseRate
    FincreaseRate = data.data
    
def state_cb(data):
    global state_data, Fkp, init_K
    state_data = data.data
    if (state_data%2) == 0:
        Fkp = 0
    if (state_data%2) == 1:
        Fkp = init_K

init_lbf = 12.34*4.44482
sum = Taz[4]+Taz[5]+Taz[6]+Taz[7]
init_K = -init_lbf/sum

rospy.init_node('depthPID',anonymous=True)

rospy.Subscriber('depth', Float32, depth_cb)
rospy.Subscriber('/PIDpara/depth', Float32, Kp_cb)
rospy.Subscriber('/state', Int32, state_cb)

pub1 = rospy.Publisher('/force/depth',Float32MultiArray,queue_size=10)

while not rospy.is_shutdown():
    pass


