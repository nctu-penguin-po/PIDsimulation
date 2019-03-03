#!/usr/bin/env python
# license removed for brevity

suminame = 'v1'
sumi_time = 10 #sec.

from static_phycal import *
import numpy as np
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray 
from std_msgs.msg import Int32MultiArray
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import time
import os

depth_data = np.zeros((1, 8))
balance_data = np.zeros((1, 8))
forward_data = np.zeros((1, 8))
turn_data = np.zeros((1, 8))

now_time = time.strftime("%Y-%m-%d.%H-%M-%S", time.gmtime())
write_f = open('/home/eric/AUVsimu/'+suminame+now_time+'.txt', 'w')

while_flag = True

write_head = ['loc', 'v', 'a', 'r1', 'r2', 'r3', 'w', 'alpha', 'F', 'T', 'BT', 'BM']
write_f.write('time\trow\tpitch\tyaw\t')
for i in range(len(write_head)):
    write_f.write(write_head[i]+'.x\t'+write_head[i]+'.y\t'+write_head[i]+'.z\t')
for i in range(8):
    write_f.write('F'+str(i)+'\t')
write_f.write('\n')

def write_file(t_stamp, auv, write_f, F, sum_F, sum_T, BF, BM):
    write_f.write(str(t_stamp)+'\t')
    writeList = [auv.get_posture(), auv.get_loc(), auv.get_v(), auv.get_a(), auv.get_r1(), auv.get_r2(), auv.get_r3(), auv.get_w(), auv.get_alpha(), sum_F, sum_T, BF, BM]
    for i in range(len(writeList)):
        ws = str(writeList[i][0])+'\t'+str(writeList[i][1])+'\t'+str(writeList[i][2])+'\t'
        write_f.write(ws)
    F_wl = [F[0][1], F[1][1], F[2][0], F[3][0], F[4][2], F[5][2], F[6][2], F[7][2]]
    for i in range(len(F)):
        ws = str(F_wl[i])+'\t'
        write_f.write(ws)
    write_f.write('\n')



def motor_cb(data):
    global pwm, v12, v16, F
    data = data.data
    tF = []
    for i in range(len(data)):
        ind = pwm.index(data[i])
        tF.append(4.4482216*v16[ind])
    F[0][1] = tF[0]
    F[1][1] = tF[1]
    F[2][0] = tF[2]
    F[3][0] = tF[3]
    F[4][2] = tF[4]
    F[5][2] = tF[5]
    F[6][2] = tF[6]
    F[7][2] = tF[7]
    
def depth_cb(data):
    global depth_data, updata_flag
    data = data.data
    for i in range(8):
        depth_data[0, i] = data[i]
    print('sum get depth')
    updata_flag = True

def balance_cb(data):
    global balance_data, updata_flag
    data = data.data
    for i in range(8):
        balance_data[0, i] = data[i]
    print('sum get balance')
    updata_flag = True

def forward_cb(data):
    global forward_data, updata_flag
    data = data.data
    for i in range(8):
        forward_data[0, i] = data[i]
    print('sum get forwad')
    updata_flag = True

def turn_cb(data):
    global turn_data, updata_flag
    data = data.data
    for i in range(8):
        turn_data[0, i] = data[i]
    print('sum get turn')
    updata_flag = True



rospy.init_node('simulation',anonymous=True)
rospy.Subscriber('/motor',Int32MultiArray, motor_cb)
rospy.Subscriber('/force/depth', Float32MultiArray, depth_cb)
rospy.Subscriber('/force/balance', Float32MultiArray, balance_cb)
rospy.Subscriber('/force/forward', Float32MultiArray, forward_cb)
rospy.Subscriber('/force/turn', Float32MultiArray, turn_cb)

pub1 = rospy.Publisher('/posture', numpy_msg(Floats),queue_size=10)
pub2 = rospy.Publisher('/depth',Float32,queue_size=10)
pub3 = rospy.Publisher('/sumi_t',Float32,queue_size=10)



time.sleep(1)

pos = np.array([0, 0, 0])
depth = 0
pub1.publish(pos)
pub2.publish(depth)
pub3.publish(t_stamp)
while while_flag:
    pass
    
write_f.close()

