#!/usr/bin/env python
# license removed for brevity
import numpy as np
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray 
from std_msgs.msg import Int32MultiArray 
from rospy.numpy_msg import numpy_msg
import time

pwm = []
v16 = []
v12 = []
voltage = 15

def read_file(filename):
    global pwm, v16, v12
    f = open(filename, 'r')
    f = f.readlines()
    for i in range(1, len(f)):
        g = f[i].split(',')
        pwm.append(int(g[0]))
        v16.append(float(g[1]))
        v12.append(float(g[2]))
    return pwm, v16, v12
   
def trans_value(v, ref, rel):
    if v > ref[-1]:
        return 1
    if v < ref[0]:
        return -1
    for i in range(len(ref)-1):
        if v > ref[i] and v < ref[i+1]:
            return rel[i]+ref[i]*((rel[i+1]-rel[i])/(ref[i+1]-ref[i]))
            

def topwm_cb(data):
    global pwm, v16, v12, voltage
    sum_data=data.data
    motor_data = []
    for i in range(len(sum_data)):
        v16f = trans_value(0.224809*sum_data[i], v16, pwm)
        v12f = trans_value(0.224809*sum_data[i], v12, pwm)
        if v16f == 1:
            motor_data.append(1100)
        elif v16f == -1:
            motor_data.append(1900)
        elif v12f == 1 or v12f == -1:
            motor_data.append(int(v16f))
        else:
            rpwm = v12f+(v16f-v12f)*(voltage-12)/4
            motor_data.append(int(round(rpwm, -1)))
    pub_data = Int32MultiArray(data = motor_data)
    pub1.publish(pub_data)

pwm, v16, v12 = read_file('thruster_force.csv')

rospy.init_node('forcetopwm',anonymous=True)

rospy.Subscriber('/force/sum', Float32MultiArray, topwm_cb)

pub1 = rospy.Publisher('/motor',Int32MultiArray,queue_size=10)

while not rospy.is_shutdown():
    pass


