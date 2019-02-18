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

turn_data = 0

'''
def turn_cb(data):
    global turn_data
    turn=data.data
    test_mat_all = np.matrix([[0], [0], [turn], [0], [0], [0]])
    result_F = T_all*test_mat_all
    pub_data = [result_F[i] for i in range(8)]
    pub_data = Float32MultiArray(data = pub_data)
    pub1.publish(pub_data)
'''

def time_cb(data):
    data = data.data
    result_F = Talphaz*0
    pub_data = [result_F[i] for i in range(8)]
    pub_data = Float32MultiArray(data = pub_data)
    pub1.publish(pub_data)

rospy.init_node('turnPID',anonymous=True)

#rospy.Subscriber('turn_command', Int32, turn_cb)
rospy.Subscriber('/sumi_t', Float32, time_cb)

pub1 = rospy.Publisher('/force/turn',Float32MultiArray,queue_size=10)

while not rospy.is_shutdown():
    pass


