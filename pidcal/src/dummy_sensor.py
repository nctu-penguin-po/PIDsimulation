#!/usr/bin/env python
# license removed for brevity
import numpy as np
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray 
from std_msgs.msg import Int32
import time

import Tkinter as tk

from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
posture_data=[0,0,0]
depth_data = 0
fc = 0
tc = 0

def B_onclick():
    global eList
    for i in range(3):
        posture_data[i] = float(eList[i].get())
    depth = float(eList[3].get())
    fc = int(eList[4].get())
    tc = int(eList[5].get())
    vol = float(eList[6].get())
    but = int(eList[7].get())
    t = float(eList[8].get())
    pos=np.array(posture_data, dtype = np.float32)
    pub1.publish(pos)
    pub2.publish(depth)
    pub3.publish(fc)
    pub4.publish(tc)
    pub5.publish(vol)
    pub6.publish(but)
    pub7.publish(t)

win = tk.Tk()
win.title('Dummy motor')

eList = []
text = ['row', 'pitch', 'yaw', 'depth', 'forward command', 'turn command', 'voltage', 'button', 'sumi_t']
for i in range(9):
    L = tk.Label(win, text = text[i]).grid(row=i, column=0)
    e = tk.Entry(win)
    eList.append(e)
    e.grid(row=i, column=1)
    e.insert('insert', 0)
b = tk.Button(win, text = 'publish', command = B_onclick).grid(row = 9, column=0)

rospy.init_node('dummy',anonymous=True)
pub1 = rospy.Publisher('/posture',numpy_msg(Floats),queue_size=10)
pub2 = rospy.Publisher('/depth',Float32,queue_size=10)
pub3 = rospy.Publisher('/forward_command',Int32,queue_size=10)
pub4 = rospy.Publisher('/turn_command',Int32,queue_size=10)
pub5 = rospy.Publisher('/voltage',Float32,queue_size=10)
pub6 = rospy.Publisher('/button',Int32,queue_size=10)
pub7 = rospy.Publisher('/sumi_t',Float32,queue_size=10)

win.mainloop()
