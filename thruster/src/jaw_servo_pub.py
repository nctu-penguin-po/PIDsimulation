#!/usr/bin/env python
# license removed for brevity
import numpy as np
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray 
from std_msgs.msg import Int32MultiArray

import Tkinter as tk


if __name__ == '__main__':
    win = tk.Tk()
    win.title('Dummy jaw servo')

    def jaw_dead_onclick():
        pub1.publish(1500)
    def jaw_open_onclick():
        pub1.publish(1700)
    def jaw_close_onclick():
        pub1.publish(1400)

    def servo_onclick():
        pub2.publish(int(e2.get()))
    def servo_init_onclick():
        e2.delete(0, 'end')
        e2.insert('insert', 2400)
        pub2.publish(2400)
    def servo_action_onclick():
        e2.delete(0, 'end')
        e2.insert('insert', 1340)
        pub2.publish(1340)

    rospy.init_node('jaw_servo',anonymous=True)
    pub1 = rospy.Publisher('jaw',Int32,queue_size=10)
    pub2 = rospy.Publisher('servo',Int32,queue_size=10)
    pub1.publish(1500)
    pub2.publish(2400)

    Lj = tk.Label(win, text = 'jaw')
    bj1 = tk.Button(win, text = 'hold', width = 20, command = jaw_dead_onclick)
    bj2 = tk.Button(win, text = 'open', width = 20, command = jaw_open_onclick)
    bj3 = tk.Button(win, text = 'close', width = 20, command = jaw_close_onclick)
    e2 = tk.Entry(win)
    b2 = tk.Button(win, text = 'servo send', width = 20, command = servo_onclick)
    bs1 = tk.Button(win, text = 'init', width = 20, command = servo_init_onclick)
    bs2 = tk.Button(win, text = 'action', width = 20, command = servo_action_onclick)

    Lj.grid(row = 0, column = 0)
    bj1.grid(row = 0, column = 1)
    bj2.grid(row = 0, column = 2)
    bj3.grid(row = 0, column = 3)
    e2.grid(row = 1, column = 0)
    b2.grid(row = 1, column = 1)
    bs1.grid(row = 1, column = 2)
    bs2.grid(row = 1, column = 3)
    e2.insert('insert', 2400)

    win.mainloop()
