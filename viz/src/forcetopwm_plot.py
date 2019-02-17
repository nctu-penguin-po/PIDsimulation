#!/usr/bin/env python
# license removed for brevity
import numpy as np
import pylab as plt
import os

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

sumforce = [0 for i in range(8)]
motorpwm = [1500 for i in range(8)]



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
    if v > ref[-1]:
        return 1
    if v < ref[0]:
        return -1
    for i in range(len(ref)-1):
        if v > ref[i] and v < ref[i+1]:
            return rel[i]+ref[i]*((rel[i+1]-rel[i])/(ref[i+1]-ref[i]))

def motor_cb(data):
    global motorpwm
    data = data.data
    for i in range(8):
        motorpwm[i] = data[i]
    updata_flag = True

def sum_cb(data):
    global sumforce
    data = data.data
    for i in range(8):
        sumforce[i] = data[i]
    updata_flag = True

dir_path = os.path.dirname(os.path.realpath(__file__))
read_file(dir_path+'/thruster_force.csv')
for i in range(len(v16)):
    v16[i] = v16[i]*4.4482216
for i in range(len(v12)):
    v12[i] = v12[i]*4.4482216

rospy.init_node('forcetopwm_plot',anonymous=True)

rospy.Subscriber('/force/sum', Float32MultiArray, sum_cb)

rospy.Subscriber('/force/motor',Int32MultiArray, motor_cb)

fig = plt.figure()
ax = fig.subplots()

p1, p2 = ax.plot(v12, pwm, 'r-', v16, pwm, 'g-')

p01, = ax.plot([sumforce[0], sumforce[1]], [motorpwm[0], motorpwm[1]], 'ro')
p23, = ax.plot([sumforce[2], sumforce[3]], [motorpwm[2], motorpwm[3]], 'go')
p4567, = ax.plot([sumforce[4], sumforce[5], sumforce[6], sumforce[7]], [motorpwm[4], motorpwm[5], motorpwm[6], motorpwm[7]], 'bo')

fig.show()

while not rospy.is_shutdown():
    time.sleep(0.5)
    p01.set_data([sumforce[0], sumforce[1]], [motorpwm[0], motorpwm[1]])
    p23.set_data([sumforce[2], sumforce[3]], [motorpwm[2], motorpwm[3]])
    p4567.set_data([sumforce[4], sumforce[5], sumforce[6], sumforce[7]], [motorpwm[4], motorpwm[5], motorpwm[6], motorpwm[7]])
    fig.canvas.draw()

