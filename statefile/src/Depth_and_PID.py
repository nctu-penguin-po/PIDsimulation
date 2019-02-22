#!/usr/bin/env python
# license removed for brevity

import numpy as np
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray 
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import time

state_data = 0

def voltage_cb(data):
    global state_data
    data = data.data
    if data < 2.5:
        state_data = 0
    elif data > 10:
        state_data = 1
    pub1.publish(state_data)

def depth_cb(data):
    global state_data
    data = data.data
    if data > 10 and state_data == 1:
        state_data = 3
    pub1.publish(state_data)

rospy.init_node('state',anonymous=True)
pub1 = rospy.Publisher('/state',Int32,queue_size=10)

rospy.Subscriber('/voltage', Float32, voltage_cb)
rospy.Subscriber('/depth', Float32, depth_cb)

while not rospy.is_shutdown():
    pass

