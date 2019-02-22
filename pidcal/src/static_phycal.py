import os
import numpy as np

auv_m = 13.04 #kg
auv_BF = 16.04 #kg
CM = [36.79, 1.88, 159.89]
BM = [66.94, -1.56, 212.19]
I=np.matrix([[581893314.86, -4187605.43, 25505252.79],
             [-4187605.43, 1089170654.30, -1972775.70],
             [25505252.79, -1972775.70, 728229808.49]])
             

auv_I = I*(1e-9)
pwm = []
v12 = []
v16 = []

def read_file(filename):
    global pwm, v16, v12
    f = open(filename, 'r')
    f = f.readlines()
    for i in range(1, len(f)):
        g = f[i].split(',')
        pwm.append(int(g[0]))
        v16.append(float(g[1]))
        v12.append(float(g[2]))
        
dir_path = os.path.dirname(os.path.realpath(__file__))
read_file(dir_path+'/thruster_force.csv')

r = [[419.35, 0, 126.57],
     [-419.35, 0, 126.57],
     [13.66, 162.85, 126.57],
     [13.66, -162.85, 126.57],
     [268.03, 162.85, 94.21],
     [-268.03, 162.85, 94.21],
     [268.03, -162.85, 94.21],
     [-268.03, -162.85, 94.21]
]

motorR = []
for i in range(len(r)):
    t = np.array([r[i][0]-CM[0], r[i][1]-CM[1], r[i][2]-CM[2]])
    t = t/1000
    motorR.append(t)

R = np.matrix([[0, 0, 1, 1, 0, 0, 0, 0], [1, 1, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 1, 1, 1, 1], [-motorR[0][2], -motorR[1][2], 0, 0, motorR[4][1], motorR[5][1], motorR[6][1], motorR[7][1]], [0, 0, motorR[2][2], motorR[3][2], -motorR[4][0], -motorR[5][0], -motorR[6][0], -motorR[7][0]], [motorR[0][0], motorR[1][0], -motorR[2][1], -motorR[3][1], 0, 0, 0, 0]])
R_pinv = np.linalg.pinv(R)

pT = np.zeros((6, 6))
pT[0, 0] = auv_m
pT[1, 1] = auv_m
pT[2, 2] = auv_m
pT[3:, 3:] = auv_I

T_all = R_pinv*pT

Tax = np.asarray(T_all[:, 0])
Tay = np.asarray(T_all[:, 1])
Taz = np.asarray(T_all[:, 2])
Talphax = np.asarray(T_all[:, 3])
Talphay = np.asarray(T_all[:, 4])
Talphaz = np.asarray(T_all[:, 5])
print(T_all)
print(Tax)
print(Tay)
print(Taz)
print(Talphax)
print(Talphay)
print(Talphaz)
