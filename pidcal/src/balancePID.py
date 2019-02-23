#!/usr/bin/env python
# license removed for brevity

from static_phycal import *
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
rowkp = 1
rowki = 1
pitchkp = 1
pitchki = 1
posture_data=[0,0,0]

rowL = [0 for i in range(10)]
pitchL = [0 for i in range(10)]

rowK = 0
pitchK = 0

#Exponentially Weighted Moving Average
def EWMA_list_create(b, N):
    a = (1-b)/(1-b**N)
    r = []
    for i in range(N):
        r.append(a*(b**i))
    return r

def pos_cb(data):
    global posture_data, rowkp, pitchkp, rowL, pitchL, EWMAlist, state_data, rowK, pitchK
    posture_data=data.data
    
    if (state_data & 2) == 0 or state_data == -1:
        pub_data = [0 for i in range(8)]
        pub_data = Float32MultiArray(data = pub_data)
        pub1.publish(pub_data)
        return

    row = posture_data[0]
    pitch = posture_data[1]

    for i in range(len(rowL)-1, 0, -1):
        rowL[i] = rowL[i-1]
    rowL[0] = row
    row_sum = 0
    for i in range(len(rowL)):
        row_sum = row_sum + rowL[i]*EWMAlist[i]
    for i in range(len(pitchL)-1, 0, -1):
        pitchL[i] = pitchL[i-1]
    pitchL[0] = pitch
    pitch_sum = 0
    for i in range(len(pitchL)):
        pitch_sum = pitch_sum + pitchL[i]*EWMAlist[i]
        
    if row < -5:
        rowK = rowK-rowkp*3
    elif row > 5:
        rowK = rowK+rowkp
    elif row_sum < -3:
        rowK = rowK-rowkp
    elif row_sum > 3:
        rowK = rowK+rowkp*0.5
        
    if pitch < -5:
        pitchK = pitchK-pitchkp*3
    elif row > 5:
        pitchK = pitchK+pitchkp
    elif pitch_sum < -3:
        pitchK = pitchK-pitchkp
    elif pitch_sum > 3:
        pitchK = pitchK+pitchkp*0.5
    #test_mat_all = np.matrix([[0], [0], [0], [-row*kp], [-pitch*kp], [0]])
    #result_F = -Talphax*(row*rowkp+row_sum*rowki) - Talphay*(pitch*pitchkp+pitch_sum*pitchki)
    result_F = Talphax*rowK + Talphay*pitchK
    result_F = result_F/10
    pub_data = [result_F[i] for i in range(8)]
    pub_data = Float32MultiArray(data = pub_data)
    pub1.publish(pub_data)

def rowPID_cb(data):
	global rowkp, rowki
	data = data.data
	rowkp = data[0]
	rowki = data[1]
	
def pitchPID_cb(data):
	global pitchkp, pitchki
	data = data.data
	pitchkp = data[0]
	pitchki = data[1]
	
def state_cb(data):
    global state_data, Fkp, rowL, pitchL, rowK, pitchK
    state_data = data.data
    if (state_data & 2) == 0:
        rowK = 0
        pitchK = 0
        for i in range(len(rowL)):
            rowL[i] = 0
        for i in range(len(pitchL)):
            pitchL[i] = 0

EWMAlist = EWMA_list_create(0.5, len(rowL))

rospy.init_node('balancePID',anonymous=True)

rospy.Subscriber('posture', numpy_msg(Floats), pos_cb)
rospy.Subscriber('/PIDpara/row', Float32MultiArray, rowPID_cb)
rospy.Subscriber('/PIDpara/pitch', Float32MultiArray, pitchPID_cb)
rospy.Subscriber('/state', Int32, state_cb)

pub1 = rospy.Publisher('/force/balance',Float32MultiArray,queue_size=10)

while not rospy.is_shutdown():
    pass


