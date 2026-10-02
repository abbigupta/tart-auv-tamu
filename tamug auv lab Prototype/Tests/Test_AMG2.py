import os
import sys
import time
import math
import numpy as np
import statistics as stat
import board
import busio
import adafruit_lsm9ds1
import matplotlib.pyplot as plt

sys.path.append(os.path.abspath('../Data/'))
'''
sensor = adafruit_lsm9ds1.LSM9DS1_I2C(busio.I2C(board.SCL, board.SDA))
'''
'''
while True:
    accel_x, accel_y, accel_z = sensor.acceleration
    mag_x, mag_y, mag_z = sensor.magnetic
    gyro_x, gyro_y, gyro_z = sensor.gyro
    temp = sensor.temperature
    os.system('clear')
    print('Acceleration (m/s^2): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(accel_x, accel_y, accel_z))
    print('Magnetometer (uT): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(mag_x*100, mag_y*100, mag_z*100))
    print('Gyroscope (degrees/sec): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(gyro_x, gyro_y, gyro_z))
    print('Temperature: {0:0.3f}C'.format(temp))
    time.sleep(0.05)
'''

'''
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
'''

N = 1000
D = np.array([[0]*3]*N)
DC = np.array([[0]*3]*N)

fid = open('../Data/AMG_Calibration_Full.txt','r')

points = fid.readline().rstrip('),').lstrip('(').split('),(')
for i in range(len(points)):
	points[i] = points[i].split(',')
	for j in range(3):
		points[i][j] = float(points[i][j])

fid.close()

D = np.array(points)
DC = np.array(points)

b = np.array([0,0,0])
s = np.array([1,1,1])
A = np.identity(3,dtype=float)
S = np.identity(3,dtype=float)

def v(p):
	return np.std(np.linalg.norm(b-D, axis=1)) - np.std(np.linalg.norm(p-D, axis=1))
def V(M):
	return np.mean((np.linalg.norm(A @ DC.T, axis=0) - R)**2) - np.mean((np.linalg.norm((A+M) @ DC.T, axis=0) - R)**2)
V2 = np.vectorize(V, signature='(m,n)->()')

b = np.mean(D, axis=0)
while (np.linalg.norm(s) > 0.00001):
	s = np.array(np.apply_along_axis(v, axis=1, arr = b+np.identity(3)*0.001)) * 1000
	b += s
	print(s)
print(f'b = {b}')
DC = D - b
R = np.linalg.norm(DC, axis=1)

while (np.linalg.norm(S) > 0.00001):
	I = np.zeros((3,3,3,3), dtype=float)
	for i in range(9):
		I[i//3, i%3, i//3, i%3] = 0.001
	S = V2(I)
	A += S
print(f'A = {A}')
DC = A @ DC.T


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
