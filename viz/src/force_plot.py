#!/usr/bin/env python
# license removed for brevity

import numpy as np
import pylab as plt
import time

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray 
from std_msgs.msg import Int32MultiArray
from rospy.numpy_msg import numpy_msg

force_data = np.zeros((4, 8))
depth_data = np.zeros((1, 8))
balance_data = np.zeros((1, 8))
forward_data = np.zeros((1, 8))
turn_data = np.zeros((1, 8))
color = ['b', 'r', 'g', 'y', 'k']

def assign_data(data, i):
    for i in range(8):
        force_data[0, i] = data[i]
    print(force_data)

def depth_cb(data):
    global depth_data
    data = data.data
    for i in range(8):
        depth_data[0, i] = data[i]
    print('sum get depth')

def balance_cb(data):
    global balance_data
    data = data.data
    for i in range(8):
        balance_data[0, i] = data[i]
    print('sum get balance')

def forward_cb(data):
    global forward_data
    data = data.data
    for i in range(8):
        forward_data[0, i] = data[i]
    print('sum get forwad')

def turn_cb(data):
    global turn_data
    data = data.data
    for i in range(8):
        turn_data[0, i] = data[i]
    print('sum get turn')

rospy.init_node('force_plot',anonymous=True)

rospy.Subscriber('/force/depth', Float32MultiArray, depth_cb)
rospy.Subscriber('/force/balance', Float32MultiArray, balance_cb)
rospy.Subscriber('/force/forward', Float32MultiArray, forward_cb)
rospy.Subscriber('/force/turn', Float32MultiArray, turn_cb)

fig = plt.figure()
ax1 = fig.add_subplot(211)
ax2 = fig.add_subplot(212)
ax1.set_ylim([-10, 10])
ax2.set_ylim([-10, 10])
width = 0.1

p1List = []

for j in range(8):
    ppList = []
    t = 0
    for i in range(4):
        p, = ax1.plot([j+i*0.1, j+i*0.1], [0, force_data[i ,j]], color[i]+'.-')
        t = t+force_data[i ,j]
        ppList.append(p)
    p, = ax1.plot([j+4*0.1, j+4*0.1], [0, t], color[4]+'.-')
    ppList.append(p)
    p1List.append(ppList)

p2List = []
for j in range(8):
    ppList = []
    t = 0
    for i in range(4):
        p, = ax2.plot([j+i*0.1, j+i*0.1], [t, t+force_data[i ,j]], color[i]+'.-')
        t = t+force_data[i ,j]
        ppList.append(p)
    p, = ax2.plot([j+4*0.1, j+4*0.1], [0, t], color[4]+'.-')
    ppList.append(p)
    p2List.append(ppList)

fig.show()

while not rospy.is_shutdown():
    force_data[0, :] = depth_data[0, :]
    force_data[1, :] = balance_data[0, :]
    force_data[2, :] = forward_data[0, :]
    force_data[3, :] = turn_data[0, :]
    time.sleep(0.5)
    for j in range(8):
        t = 0
        for i in range(4):
            p1List[j][i].set_data([j+i*0.1, j+i*0.1], [0, force_data[i ,j]])
            t = t+force_data[i ,j]
        p1List[j][4].set_data([j+4*0.1, j+4*0.1], [0, t])

    for j in range(8):
        t = 0
        for i in range(4):
            p2List[j][i].set_data([j+i*0.1, j+i*0.1], [t, t+force_data[i ,j]])
            t = t+force_data[i ,j]
        p2List[j][4].set_data([j+4*0.1, j+4*0.1], [0, t])
    fig.canvas.draw()

