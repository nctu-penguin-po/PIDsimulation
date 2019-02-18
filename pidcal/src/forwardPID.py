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
kp = 0.1
forward_data = 0

'''
def forward_cb(gdata):
    global forward_data, suT
    forward_data=gdata.data
    #test_mat_all = np.matrix([[forward_data], [0], [0], [0], [0], [0]])
    result_F = T_all*test_mat_all
    pub_data = [result_F[i] for i in range(8)]
    pub_data = Float32MultiArray(data = pub_data)
    pub1.publish(pub_data)
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

rospy.init_node('forwardPID',anonymous=True)

#rospy.Subscriber('forward_command', Int32, forward_cb)
rospy.Subscriber('sumi_t', Float32, time_cb)

pub1 = rospy.Publisher('/force/forward',Float32MultiArray,queue_size=10)

while not rospy.is_shutdown():
    pass


