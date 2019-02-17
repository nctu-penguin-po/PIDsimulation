#!/usr/bin/env python
# license removed for brevity
import numpy as np
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray 
from std_msgs.msg import Int32MultiArray
from rospy.numpy_msg import numpy_msg
import time

depth_data = 0

I=np.matrix([[0.43040321996,0.00046648212,0.00015004917],
             [0.00046648212,0.4102308932,-0.01892857864],
             [0.00015004917,-0.01892857864,0.13371635255]])
r = [[4, -1, -0.5],
     [-5, -1, -0.5],
     [1, -3, -0.5],
     [1, 1, -0.5],
     [3, -3, -1],
     [-4, -3, -1],
     [3, 1, -1],
     [-4, 1, -1]]
m = 100

R = np.matrix([[0, 0, 1, 1, 0, 0, 0, 0], [1, 1, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 1, 1, 1, 1], [-r[0][2], -r[1][2], 0, 0, r[4][1], r[5][1], r[6][1], r[7][1]], [0, 0, r[2][2], r[3][2], -r[4][0], -r[5][0], -r[6][0], -r[7][0]], [r[0][0], r[1][0], -r[2][1], -r[3][1], 0, 0, 0, 0]])
R_pinv = np.linalg.pinv(R)

pT = np.zeros((6, 6))
pT[0, 0] = m
pT[1, 1] = m
pT[2, 2] = m
pT[3:, 3:] = I

T_all = R_pinv*pT

def depth_cb(data):
    global depth_data
    depth_data=data.data
    
    if depth_data < 10:
        az = 0.1
    elif depth_data > 20:
        az = -0.1
    else:
        az = 0
    test_mat_all = np.matrix([[0], [0], [az], [0], [0], [0]])
    result_F = T_all*test_mat_all
    pub_data = [result_F[i] for i in range(8)]
    pub_data = Float32MultiArray(data = pub_data)
    pub1.publish(pub_data)


rospy.init_node('depthPID',anonymous=True)

rospy.Subscriber('depth', Float32, depth_cb)

pub1 = rospy.Publisher('/force/depth',Float32MultiArray,queue_size=10)

while not rospy.is_shutdown():
    pass


