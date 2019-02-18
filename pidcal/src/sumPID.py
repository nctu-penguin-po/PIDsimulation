#!/usr/bin/env python
# license removed for brevity

from static_phycal import *
import os 
import numpy as np
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray 
from std_msgs.msg import Int32MultiArray
from rospy.numpy_msg import numpy_msg
import time

updata_flag1 = False
updata_flag2 = False
updata_flag3 = False
updata_flag4 = False

voltage = 15
depth_data = np.zeros((1, 8))
balance_data = np.zeros((1, 8))
forward_data = np.zeros((1, 8))
turn_data = np.zeros((1, 8))

def trans_value(v, ref, rel):
    if abs(v) < 0.01:
        return 1500
    if v > ref[-1]:
        return 1
    if v < ref[0]:
        return -1
    for i in range(len(ref)-1):
        if v > ref[i] and v < ref[i+1]:
            print(ref[i], rel[i])
            return rel[i]+(rel[i+1]-rel[i])*((v-ref[i])/(ref[i+1]-ref[i]))

def depth_cb(data):
    global depth_data, updata_flag1
    data = data.data
    for i in range(8):
        depth_data[0, i] = data[i]
    print('sum get depth')
    updata_flag1 = True

def balance_cb(data):
    global balance_data, updata_flag2
    data = data.data
    for i in range(8):
        balance_data[0, i] = data[i]
    print('sum get balance')
    updata_flag2 = True

def forward_cb(data):
    global forward_data, updata_flag3
    data = data.data
    for i in range(8):
        forward_data[0, i] = data[i]
    print('sum get forwad')
    updata_flag3 = True

def turn_cb(data):
    global turn_data, updata_flag4
    data = data.data
    for i in range(8):
        turn_data[0, i] = data[i]
    print('sum get turn')
    updata_flag4 = True

rospy.init_node('sumPID',anonymous=True)

rospy.Subscriber('/force/depth', Float32MultiArray, depth_cb)
rospy.Subscriber('/force/balance', Float32MultiArray, balance_cb)
rospy.Subscriber('/force/forward', Float32MultiArray, forward_cb)
rospy.Subscriber('/force/turn', Float32MultiArray, turn_cb)

pub1 = rospy.Publisher('/force/sum',Float32MultiArray,queue_size=10)
pub2 = rospy.Publisher('/force/motor',Int32MultiArray,queue_size=10)

print(v16)
print(v12)

while not rospy.is_shutdown():
    if updata_flag1 == True and updata_flag2 == True and updata_flag3 == True and updata_flag4 == True:
        updata_flag1 = False
        updata_flag2 = False
        updata_flag3 = False
        updata_flag4 = False

        force_data = depth_data + balance_data + forward_data + turn_data
        sum_data = list(force_data)[0]
        print(sum_data)
        pub_data = Float32MultiArray(data = sum_data)
        pub1.publish(pub_data)
        motor_data = []
        for i in range(len(sum_data)):
            v16f = trans_value(0.224809*sum_data[i], v16, pwm)
            v12f = trans_value(0.224809*sum_data[i], v12, pwm)
            print(v16f)
            print(v12f)
            if v16f == 1:
                motor_data.append(1900)
            elif v16f == -1:
                motor_data.append(1100)
            elif v12f == 1 or v12f == -1:
                motor_data.append(int(round(v16f, -1)))
            else:
                rpwm = v12f+(v16f-v12f)*(voltage-12)/4
                motor_data.append(int(round(rpwm, -1)))
        pub_data = Int32MultiArray(data = motor_data)
        pub2.publish(pub_data)


