#!/usr/bin/env python
# license removed for brevity

from static_phycal import *
import numpy as np
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray 
from std_msgs.msg import Int32
from rospy.numpy_msg import numpy_msg
import time

#suT = 0

forwardFlag = 0
state_data = 0

def forward_cb(gdata):
    if forwardFlag == 0 or state == 0 or state == -1:
        pub_data = [0 for i in range(8)]
        pub_data = Float32MultiArray(data = pub_data)
        pub1.publish(pub_data)
        return
        
    #test_mat_all = np.matrix([[forward_data], [0], [0], [0], [0], [0]])
    result_F = Tax*ax*forwardFlag
    pub_data = [result_F[i] for i in range(8)]
    pub_data = Float32MultiArray(data = pub_data)
    pub1.publish(pub_data)

def state_cb(data):
    global state_data
    state_data = data.data

def turnflag_cb(data):
    global forwardFlag
    data = data.data
    if data[2] == 0 and data[3] == 0:
        forwardFlag = data[2]
    else:
        forwardFlag = 1
'''
def time_cb(data):
    data = data.data
    if data > 9:
        ax = 0.1
    else:
        ax = 0
    result_F = Tax*ax
    pub_data = [result_F[i] for i in range(8)]
    pub_data = Float32MultiArray(data = pub_data)
    pub1.publish(pub_data)
'''

#F = 6.23N = 1.4lbf = pwm1600
Kp = (Tax[2]+Tax[3])/(2*6.23)

rospy.init_node('forwardPID',anonymous=True)

rospy.Subscriber('forward_command', Int32, forward_cb)
rospy.Subscriber('/state', Int32, state_cb)
rospy.Subscriber('/flag/PIDturn', Int32MultiArray, turnflag_cb)
#rospy.Subscriber('sumi_t', Float32, time_cb)

pub1 = rospy.Publisher('/force/forward',Float32MultiArray,queue_size=10)

while not rospy.is_shutdown():
    pass


