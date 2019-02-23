#!/usr/bin/env python
# license removed for brevity

import numpy as np
import pylab as plt
import time

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from std_msgs.msg import Float32MultiArray 
from std_msgs.msg import Int32MultiArray
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

import Tkinter as tk

def FtoS(v):
    if abs(v) > 1:
        return '%5f' % v
    if abs(v) > 0.0001:
        return '%.5f' % v
    return '%.2e' % v

def ItoS(v):
    return '%5d' % v

def handle_updateI(data, t):
    global labelList
    data = data.data
    for i in range(8):
        labelList[i][t].config(text = ItoS(data[i]))

def handle_update(data, t):
    global labelList
    data = data.data
    for i in range(8):
        if abs(data[i]) < 0.5:
            b = 'white'
        elif abs(data[i]) > 20:
            b = 'red'
        elif data[i] > 0:
            b = 'green'
        else:
            b = 'blue'
        labelList[i][t].config(text = FtoS(data[i]), bg = b)

def fdepth_cb(data):
    handle_update(data, 0)


def fbalance_cb(data):
    handle_update(data, 1)
        
def fforward_cb(data):
    handle_update(data, 2)
        
def fturn_cb(data):
    handle_update(data, 3)
    
def fsum_cb(data):
    handle_update(data, 4)
    
def fmotor_cb(data):
    handle_updateI(data, 5)
    
def depth_cb(data):
    global inforLabelList
    data = data.data
    inforLabelList[0].config(text = str(data))

def posture_cb(data):
    global inforLabelList
    data = data.data
    s = 'row:'+FtoS(data[0])+'\npitch:'+FtoS(data[1])+'\nyaw: '+FtoS(data[2])
    inforLabelList[1].config(text = s)

def voltage_cb(data):
    global inforLabelList
    data = data.data
    inforLabelList[2].config(text = str(data))
    
def time_cb(data):
    global inforLabelList
    data = data.data
    inforLabelList[3].config(text = FtoS(data))

def state_cb(data):
    global inforLabelList
    data = data.data
    inforLabelList[4].config(text = ItoS(data))

rospy.Subscriber('/depth', Float32, depth_cb)
rospy.Subscriber('/posture', numpy_msg(Floats), posture_cb)
rospy.Subscriber('/voltage', Float32, voltage_cb)
rospy.Subscriber('/sumi_t', Float32, time_cb)
rospy.Subscriber('/state', Int32, state_cb)

forcePubList = []
forcePubName = ['/force/depth', '/force/balance', '/force/forward', '/force/turn', '/force/sum', '/motor']
forcePubClass = [Float32MultiArray, Float32MultiArray, Float32MultiArray, Float32MultiArray, Float32MultiArray, Int32MultiArray]
forcePubFunc = [fdepth_cb, fbalance_cb, fforward_cb, fturn_cb, fsum_cb, fmotor_cb]

rospy.init_node('dashboard',anonymous=True)

for i in range(6):
    p = rospy.Subscriber(forcePubName[i], forcePubClass[i], forcePubFunc[i])
    forcePubList.append(p)
    
win = tk.Tk()
win.title('Dashboard')
win.geometry('800x800')

placeListx = [2, 2, 0, 4, 1, 1, 3, 3]
placeListy = [0, 8, 4, 4, 2, 6, 2, 6]
labelList = []
for i in range(8):
    sublabelList = []
    f = tk.Frame(win)
    f.place(x = placeListx[i]*100, y = placeListy[i]*80, anchor = 'nw')
    l = tk.Label(f, width = 20, text = 'motor'+str(i), font=('Arial', 12), bg = 'black', fg = 'white')
    l.pack(side='top')
    fl = tk.Frame(f)
    fl.pack(side='left')
    fr = tk.Frame(f)
    fr.pack(side='right')
    
    for j in range(4):
        l = tk.Label(fl, width = 10, text = '-', font=('Arial', 12))
        l.pack()
        sublabelList.append(l)
    for j in range(2):
        l = tk.Label(fr, width = 10, height=2, text = '-', font=('Arial', 12))
        l.pack()
        sublabelList.append(l)
    labelList.append(sublabelList)
    
f2 = tk.Frame(win)
f2.place(x = 600, y = 0, anchor = 'nw')

inforLabelList = []
topicList = ['Depth', 'Posture', 'voltage', 'sumi_t', 'state']
heightList = [1, 3, 1, 1, 1]
for i in range(len(topicList)):
    l = tk.Label(f2, text = topicList[i], bg = 'black', fg = 'white', font=('Arial', 12))
    l.pack()
    L = tk.Label(f2, text = '-', height = heightList[i], font=('Arial', 12))
    L.pack()
    inforLabelList.append(L)

win.mainloop()
