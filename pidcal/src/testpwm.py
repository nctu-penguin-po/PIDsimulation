import pylab as plt
import numpy as np

pwm = []
v16 = []
v12 = []
voltage = 15


def read_file(filename):
    global pwm, v16, v12
    f = open(filename, 'r')
    f = f.readlines()
    for i in range(1, len(f)):
        g = f[i].split(',')
        pwm.append(int(g[0]))
        v16.append(float(g[1]))
        v12.append(float(g[2]))
   
def trans_value(v, ref, rel):
    if abs(v) < 0.01:
        return 1500
    if v > ref[-1]:
        return 1
    if v < ref[0]:
        return -1
    for i in range(len(ref)-1):
        if v > ref[i] and v < ref[i+1]:
            return rel[i]+(rel[i+1]-rel[i])*((v-ref[i])/(ref[i+1]-ref[i]))


def get_pwm(f, v):
    global pwm, v16, v12
    voltage = v
    v16f = trans_value(0.224809*f, v16, pwm)
    v12f = trans_value(0.224809*f, v12, pwm)
    if v16f == 1:
        motor = 1900
    elif v16f == -1:
        motor = 1100
    elif v12f == 1 or v12f == -1:
        motor = int(round(v16f, -1))
    else:
        rpwm = v12f+(v16f-v12f)*(voltage-12)/4
        motor = int(round(rpwm, -1))
    return motor


read_file('thruster_force.csv')

test_data = np.arange(-50, 60, 0.01)
print(test_data)

test_r = []
for i in range(len(test_data)):
    r = get_pwm(test_data[i], 15)
    test_r.append(r)
print(test_r)

for i in range(len(v12)):
    v12[i] = v12[i]*4.4482216
for i in range(len(v16)):
    v16[i] = v16[i]*4.4482216
plt.plot(v16, pwm, 'b', v12, pwm, 'r', list(test_data), test_r, 'k')
plt.show()
