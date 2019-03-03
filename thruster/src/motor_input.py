#!/usr/bin/env python
# license removed for brevity
import numpy as np
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray 
from std_msgs.msg import Int32MultiArray
import time

import Tkinter as tk

from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
posture_data=[0,0,0]
motor_data=[0,0,0,0,0,0,0,0]


def stop_onclick():
    stop_list = [1500 for i in range(8)]
    motorrrr=Int32MultiArray(data = stop_list)
    pub1.publish(motorrrr)

#if __name__ == '__main__':
def B_onclick():
    global eList
    for i in range(8):
        motor_data[i] = int(eList[i].get())
    motorrrr=Int32MultiArray(data = motor_data)
    pub1.publish(motorrrr)

win = tk.Tk()
win.title('Dummy motor')

eList = []
for i in range(8):
    L = tk.Label(win, text = 'motor'+str(i)).grid(row=i, column=0)
    e = tk.Entry(win)
    eList.append(e)
    e.grid(row=i, column=1)
    e.insert('insert', 1500)
b = tk.Button(win, text = 'motor', command = B_onclick).grid(row = 8, column=0)
bs = tk.Button(win, text = 'all stop', command = stop_onclick).grid(row = 8, column=1)

rospy.init_node('IMU2motor',anonymous=True)
pub1 = rospy.Publisher('motor',Int32MultiArray,queue_size=10)
#print("posture666")



for i in range(len(motor_data)):
    motor_data[i] = 1500
time.sleep(1)
motorrrr=Int32MultiArray(data = motor_data)
pub1.publish(motorrrr)

win.mainloop()
