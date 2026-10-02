import os
import sys
import time
import math
import numpy as np
import statistics
import board
import busio
import adafruit_lsm9ds1
import matplotlib.pyplot as plt

sys.path.append(os.path.abspath('../Data/'))

sensor = adafruit_lsm9ds1.LSM9DS1_I2C(busio.I2C(board.SCL, board.SDA))


mx=[]; my=[]; mz=[]; ax,ay,az=[],[],[]
for itick in range(20*30):
    accel_x, accel_y, accel_z = sensor.acceleration
    mag_x, mag_y, mag_z = sensor.magnetic
    gyro_x, gyro_y, gyro_z = sensor.gyro
    temp = sensor.temperature
    os.system('clear')
    print('Acceleration (m/s^2): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(accel_x, accel_y, accel_z))
    print('Magnetometer (uT): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(mag_x*100, mag_y*100, mag_z*100))
    print('Gyroscope (degrees/sec): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(gyro_x, gyro_y, gyro_z))
    print('Temperature: {0:0.3f}C'.format(temp))
    mx.append(mag_x*100)
    my.append(mag_y*100)
    mz.append(mag_z*100)
    ax.append(accel_x)
    ay.append(accel_y)
    az.append(accel_z)
    time.sleep(0.05)

print(f'mean mag_x {np.mean(mx):.2f}')
print(f'mean mag_y {np.mean(my):.2f}')
print(f'mean mag_z {np.mean(mz):.2f}')

print(f'mean acc_x {np.mean(ax):.2f}')
print(f'mean acc_y {np.mean(ay):.2f}')
print(f'mean acc_z {np.mean(az):.2f}')

'''
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
'''

N = 1000
D = [[],[],[]]
DC = [[],[],[]]
'''
for i in range(N):
	x, y, z = sensor.magnetic
	D[0].append(x*100)
	D[1].append(y*100)
	D[2].append(z*100)
	DC[0].append(0)
	DC[1].append(0)
	DC[2].append(0)
	time.sleep(0.05)
	os.system('clear')
	print(f'{i+1}/{N}')
'''
'''
fid = open('../Data/AMG_Calibration_Full.txt','r')
points = fid.readline().rstrip('),').lstrip('(').split('),(')
for i in range(len(points)):
	points[i] = points[i].split(',')
	for j in range(3):
		points[i][j] = round(float(points[i][j]),6)
print(points)
fid.close()

b = [0,0,0]
s = [1,1,1]

R = 40
A = [[1,0,0],[0,1,0],[0,0,1]]
S = [[1,0,0],[0,1,0],[0,0,1]]

def h(L):
	su = 0.0
	for l in L:
		su += l**2
	return math.sqrt(su)

def H(LL):
	su = 0.0
	for L in LL:
		for l in L:
			su += l**2
	return math.sqrt(su)

def T(p, M):
	return [p[0]*M[0][0]+p[1]*M[0][1]+p[2]*M[0][2],p[0]*M[1][0]+p[1]*M[1][1]+p[2]*M[1][2],p[0]*M[2][0]+p[1]*M[2][1]+p[2]*M[2][2]]
'''
'''
def v(p):
	db = []
	dp = []
	for i in range(N):
		db.append(math.hypot(b[0]-DC[0][i],b[1]-DC[1][i],b[2]-DC[2][i]))
		dp.append(math.hypot(p[0]-DC[0][i],p[1]-DC[1][i],p[2]-DC[2][i]))
	return statistics.stdev(db) - statistics.stdev(dp)

def Kb():
	global b
	global s
	b0 = [b[0]+0.001,b[1],b[2]]
	b1 = [b[0],b[1]+0.001,b[2]]
	b2 = [b[0],b[1],b[2]+0.001]
	s = [v(b0)*1000,v(b1)*1000,v(b2)*1000]
	b = [b[0]+s[0],b[1]+s[1],b[2]+s[2]]
'''

'''
def V(M):
	d2A = []
	d2M = []
	for i in range(N):
		d2A.append((h(T([DC[0][i],DC[1][i],DC[2][i]], A))-R)**2)
		d2M.append((h(T([DC[0][i],DC[1][i],DC[2][i]], M))-R)**2)
	return statistics.mean(d2A) - statistics.mean(d2M)

def KA():
	global A
	global S
	A0 = [[A[0][0]+0.001,A[0][1],A[0][2]],A[1],A[2]]
	A1 = [[A[0][0],A[0][1]+0.001,A[0][2]],A[1],A[2]]
	A2 = [[A[0][0],A[0][1],A[0][2]+0.001],A[1],A[2]]
	A3 = [A[0],[A[1][0]+0.001,A[1][1],A[1][2]],A[2]]
	A4 = [A[0],[A[1][0],A[1][1]+0.001,A[1][2]],A[2]]
	A5 = [A[0],[A[1][0],A[1][1],A[1][2]+0.001],A[2]]
	A6 = [A[0],A[1],[A[2][0]+0.001,A[2][1],A[2][2]]]
	A7 = [A[0],A[1],[A[2][0],A[2][1]+0.001,A[2][2]]]
	A8 = [A[0],A[1],[A[2][0],A[2][1],A[2][2]+0.001]]
	S = [[V(A0),V(A1),V(A2)],[V(A3),V(A4),V(A5)],[V(A6),V(A7),V(A8)]]
	A = [[A[0][0]+S[0][0],A[0][1]+S[0][1],A[0][2]+S[0][2]],[A[1][0]+S[1][0],A[1][1]+S[1][1],A[1][2]+S[1][2]],[A[2][0]+S[2][0],A[2][1]+S[2][1],A[2][2]+S[2][2]]]
'''

'''
# Find b
for i in range(3):
	b[i] = statistics.mean(D[i])
while (h(s) > 0.0001):
	print(h(s))
	Kb()
print(f'b = [{b[0]:.4f}, {b[1]:.4f}, {b[2]:.4f}]')

# Apply b
for i in range(N):
	DC[0][i] = D[0][i] - b[0]
	DC[1][i] = D[1][i] - b[1]
	DC[2][i] = D[2][i] - b[2]
'''

'''
# Find A
while (H(S) > 0.0001):
	KA()
print(f'A = [[{A[0][0]:.4f}, {A[0][1]:.4f}, {A[0][2]:.4f}]')
print(f'     [{A[1][0]:.4f}, {A[1][1]:.4f}, {A[1][2]:.4f}]')
print(f'     [{A[2][0]:.4f}, {A[2][1]:.4f}, {A[2][2]:.4f}]]')

# Apply A
for i in range(N):
	dc = T([DC[0][i],DC[1][i],DC[2][i]], A)
	DC[0][i] = dc[0]
	DC[1][i] = dc[1]
	DC[2][i] = dc[2]
'''

'''
ax.scatter(D[0], D[1], D[2], color='r')
ax.scatter(DC[0], DC[1], DC[2], color='g')
ax.scatter([b[0],0], [b[1],0], [b[2],0], color='b')
plt.show()
'''

'''
print(statistics.stdev(D[0]))
print(statistics.stdev(D[1]))
print(statistics.stdev(D[2]))
'''
