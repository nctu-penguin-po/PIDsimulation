#!/usr/bin/env python
# license removed for brevity

from static_phycal import *
import numpy as np
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray 
from std_msgs.msg import Int32MultiArray
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import time

kp = 0
posture_data=[0,0,0]

def pos(data):
    global posture_data, kp
    posture_data=data.data
    row = posture_data[0]
    pitch = posture_data[1]
    #test_mat_all = np.matrix([[0], [0], [0], [-row*kp], [-pitch*kp], [0]])
    result_F = -Talphax*row*kp - Talphay*pitch*kp
    pub_data = [result_F[i] for i in range(8)]
    pub_data = Float32MultiArray(data = pub_data)
    pub1.publish(pub_data)


rospy.init_node('balancePID',anonymous=True)

rospy.Subscriber('posture', numpy_msg(Floats), pos)

pub1 = rospy.Publisher('/force/balance',Float32MultiArray,queue_size=10)

while not rospy.is_shutdown():
    pass


