#!/usr/bin/env python
# license removed for brevity

from static_phycal import *
import numpy as np
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray 
from std_msgs.msg import Int32MultiArray
from rospy.numpy_msg import numpy_msg
import time

depth_data = 0

def depth_cb(data):
    global depth_data
    depth_data=data.data
    
    if depth_data < 10:
        az = -1
    elif depth_data > 20:
        az = 1
    else:
        az = 0
    #test_mat_all = np.matrix([[0], [0], [az], [0], [0], [0]])
    result_F = Taz*az
    pub_data = [result_F[i] for i in range(8)]
    pub_data = Float32MultiArray(data = pub_data)
    pub1.publish(pub_data)


rospy.init_node('depthPID',anonymous=True)

rospy.Subscriber('depth', Float32, depth_cb)

pub1 = rospy.Publisher('/force/depth',Float32MultiArray,queue_size=10)

while not rospy.is_shutdown():
    pass


