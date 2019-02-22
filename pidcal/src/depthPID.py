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
depthL = [for 0 in range(10)]

Fkp = 0
FincreaseRate = 0.5

def depth_cb(data):
    global depthL, Fkp, FincreaseRate, depth_data
    depth_data=data.data
    if (state%2) == 0 or state == -1:
        pub_data = [0 for i in range(8)]
        pub_data = Float32MultiArray(data = pub_data)
        pub1.publish(pub_data)
        return
    
    if depth_data < 20:
        Fkp = Fkp+FincreaseRate
    elif depth_data > 40:
        Fkp = Fkp+FincreaseRate

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


