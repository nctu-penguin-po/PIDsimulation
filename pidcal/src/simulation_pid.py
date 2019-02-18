#!/usr/bin/env python
# license removed for brevity

import numpy as np
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray 
from std_msgs.msg import Int32MultiArray
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import time
import os

pwm = []
v16 = []
v12 = []

def read_file(filename):
    global pwm, v16, v12
    f = open(filename, 'r')
    f = f.readlines()
    for i in range(1, len(f)):
        g = f[i].split(',')
        pwm.append(int(g[0]))
        v16.append(float(g[1]))
        v12.append(float(g[2]))

class Rigidbody3D:
  def __init__(self, m, I, t_step, init_loc = np.array([0, 0, 0]), init_v = np.array([0, 0, 0]), initrs = [np.array([1, 0, 0]), np.array([0, 1, 0]), np.array([0, 0, 1])], init_w = np.array([0, 0, 0])):
    self.loc = init_loc
    self.v = init_v
    self.a = np.array([0, 0, 0])
    self.r1 = initrs[0]
    self.r2 = initrs[1]
    self.r3 = initrs[2]
    self.w = init_w
    self.alpha = np.array([0, 0, 0])
    self.m = m
    self.I = I
    self.t_step = t_step
    self.inv_I = np.linalg.inv(I)
    
    
  def get_loc(self):
    return self.loc
  def get_v(self):
    return self.v
  def get_a(self):
    return self.a
  def get_r1(self):
    return self.r1
  def get_r2(self):
    return self.r2
  def get_r3(self):
    return self.r3
  def get_w(self):
    return self.w
  def get_alpha(self):
    return self.alpha
  def get_posture(self):
    rt = self.r1 + self.r2
    cos_yaw = (rt[0]+rt[1])/(np.sqrt(rt[0]*rt[0]+rt[1]*rt[1])*np.sqrt(2))
    yaw = np.arccos(cos_yaw)*180/np.pi
    if rt[0] > rt[1]:
      yaw = -yaw
    rt = self.r2 + self.r3
    cos_row = (rt[1]+rt[2])/(np.sqrt(rt[1]*rt[1]+rt[2]*rt[2])*np.sqrt(2))
    row = np.arccos(cos_row)*180/np.pi
    if rt[1] > rt[2]:
      row = -row
    rt = self.r3 + self.r1
    cos_pitch = (rt[2]+rt[0])/(np.sqrt(rt[2]*rt[2]+rt[0]*rt[0])*np.sqrt(2))
    pitch = np.arccos(cos_pitch)*180/np.pi
    if rt[2] > rt[0]:
      pitch = -pitch
    return row, pitch, yaw
  
  def set_F(self, F):
    self.F = F
  def set_T(self, T):
    self.T = T
    
  def update(self):
    self.a = self.F/self.m
    dv = self.a*self.t_step
    self.v = self.v+dv
    dloc = self.v*self.t_step
    self.loc = self.loc+dloc
    
    T = self.T.reshape((3, 1))
    alpha = self.inv_I*T
    self.alpha = np.squeeze(np.asarray(alpha))
    dw = self.alpha*self.t_step
    self.w = self.w+dw
    
    vr1 = np.cross(self.w, self.r1)
    vr2 = np.cross(self.w, self.r2)
    vr3 = np.cross(self.w, self.r3)
    dr1 = vr1*self.t_step
    dr2 = vr2*self.t_step
    dr3 = vr3*self.t_step
    self.r1 = self.r1+dr1
    self.r2 = self.r2+dr2
    self.r3 = self.r3+dr3
    self.r1 = self.r1/np.linalg.norm(self.r1)
    self.r2 = self.r2/np.linalg.norm(self.r2)
    self.r3 = self.r3/np.linalg.norm(self.r3)
    t1 = abs(np.dot(self.r1, self.r2))
    t2 = abs(np.dot(self.r2, self.r3))
    t3 = abs(np.dot(self.r3, self.r1))
    if t1 == min(t1, t2, t3):
      self.r3 = np.cross(self.r1, self.r2)
    elif t2 == min(t1, t2, t3):
      self.r1 = np.cross(self.r2, self.r3)
    else:
      self.r2 = np.cross(self.r3, self.r1)

F = [np.array([0, 0, 0]) for i in range()]
m = 14.14
I=np.matrix([[0.43040321996,0.00046648212,0.00015004917],
             [0.00046648212,0.4102308932,-0.01892857864],
             [0.00015004917,-0.01892857864,0.13371635255]])
t_stamp = 0
t_sample = 0.01
auv = Rigidbody3D(m, I, t_sample)

r = [[419.35, 0, 126.57],
     [-419.35, 0, 126.57],
     [13.66, 162.85, 126.57],
     [13.66, -162.85, 126.57],
     [268.03, 162.85, 94.21],
     [-268.03, 162.85, 94.21],
     [268.03, -162.85, 94.21],
     [-268.03, -162.85, 94.21]
]
CM = [20.79, 3.87, 154.89]
motorR = []
for i in range(len(r)):
    t = np.array([r[i][0]-CM[0], r[i][1]-CM[1], r[i][2]-CM[2]])
    motorR.append(t)

def call_update():
    global auv, F, t_sample, t_stamp
    totalF = np.array([0, 0, 0])
    totalT = np.array([0, 0, 0])
    for i in range(len(F)):
        totalF = totalF+F[i]
        totalT = totalT+(np.cross(r[i], F[i]))
    auv.set_F(totalF)
    auv.set_T(totalT)
    for i in range(10):
        t_stamp = t_stamp+t_sample
        auv.update()
    post = auv.get_posture()
    pos=np.array([float(post[0]),float(post[1]), float(post[2])], dtype = np.float32)
    depth = -(auv.get_loc()[2])
    pub1.publish(pos)
    pub2.publish(depth)

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
    call_update()

dir_path = os.path.dirname(os.path.realpath(__file__))
read_file(dir_path+'/thruster_force.csv')

rospy.init_node('simulation',anonymous=True)
rospy.Subscriber('/force/motor',Int32MultiArray, motor_cb)

pub1 = rospy.Publisher('/posture', numpy_msg(Floats),queue_size=10)
pub2 = rospy.Publisher('depth',Float32,queue_size=10)
while not rospy.is_shutdown():
    pass

