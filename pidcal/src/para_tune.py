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

def B_onclick():
    global eList
    row_para = [float(eList[0].get()), float(eList[1].get())]
    pitch_para = [float(eList[2].get()), float(eList[3].get())]
    turnKp = float(eList[4].get())
    depthKp = float(eList[5].get())
    pub1.publish(row_para)
    pub2.publish(pitch_para)
    pub3.publish(turnKp)
    pub4.publish(depthKp)

win = tk.Tk()
win.title('Dummy motor')

eList = []
text = ['rowKp', 'rowKi', 'pitchKp', 'pitchKi', 'turnKp', 'depthKp']
for i in range(len(text)):
    L = tk.Label(win, text = text[i]).grid(row=i, column=0)
    e = tk.Entry(win)
    eList.append(e)
    e.grid(row=i, column=1)
    e.insert('insert', 0)
b = tk.Button(win, text = 'publish', command = B_onclick).grid(row = len(text), column=0)

rospy.init_node('para_tune',anonymous=True)
pub1 = rospy.Publisher('/PIDpara/row',Float32MultiArray,queue_size=10)
pub2 = rospy.Publisher('/PIDpara/pitch',Float32MultiArray,queue_size=10)
pub3 = rospy.Publisher('/PIDpara/turn',Float32,queue_size=10)
pub4 = rospy.Publisher('/PIDpara/depth',Int32,queue_size=10)
#pub5 = rospy.Publisher('/voltage',Float32,queue_size=10)
#pub6 = rospy.Publisher('/button',Int32,queue_size=10)
#pub7 = rospy.Publisher('/sumi_t',Float32,queue_size=10)

win.mainloop()
