#!/usr/bin/env python
# license removed for brevity

import os 
import numpy as np
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray 
from std_msgs.msg import Int32MultiArray
from rospy.numpy_msg import numpy_msg
import time

updata_flag = False

pwm = []
v16 = []
v12 = []
voltage = 15
depth_data = np.zeros((1, 8))
balance_data = np.zeros((1, 8))
forward_data = np.zeros((1, 8))
turn_data = np.zeros((1, 8))

def read_file(filename):
    global pwm, v16, v12
    f = open(filename, 'r')
    f = f.readlines()
    for i in range(1, len(f)):
        g = f[i].split(',')
        pwm.append(int(g[0]))
        v16.append(float(g[1]))
        v12.append(float(g[2]))
   
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
    global depth_data, updata_flag
    data = data.data
    for i in range(8):
        depth_data[0, i] = data[i]
    print('sum get depth')
    updata_flag = True

def balance_cb(data):
    global balance_data, updata_flag
    data = data.data
    for i in range(8):
        balance_data[0, i] = data[i]
    print('sum get balance')
    updata_flag = True

def forward_cb(data):
    global forward_data, updata_flag
    data = data.data
    for i in range(8):
        forward_data[0, i] = data[i]
    print('sum get forwad')
    updata_flag = True

def turn_cb(data):
    global turn_data, updata_flag
    data = data.data
    for i in range(8):
        turn_data[0, i] = data[i]
    print('sum get turn')
    updata_flag = True

dir_path = os.path.dirname(os.path.realpath(__file__))
read_file(dir_path+'/thruster_force.csv')
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
    if updata_flag == True:
        updata_flag = False
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


