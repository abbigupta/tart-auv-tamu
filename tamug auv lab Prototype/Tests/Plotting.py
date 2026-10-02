import matplotlib.pyplot as plt
import numpy as np

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

print(dir(ax))

plt.show()

'''
A = np.array([[1,0,0],
			  [0,1,0],
			  [0,0,1]])

b = np.array([0,0,0])

rawData = np.genfromtxt('../Data/RawMagReadings.txt', delimiter='\t')

N = len(rawData)
calibData = np.zeros((N, 3), dtype='float')
for i in range(N):
	currMeas = np.array([rawData[i, 0], rawData[i, 1], rawData[i, 2]])
	calibData[i, :] = A @ (currMeas - b)

plt.figure()
plt.plot(rawData[:, 0], rawData[:, 1], 'b*', label='Raw Meas.')
plt.plot(calibData[:, 0], calibData[:, 1], 'r*', label='Calibrated Meas.')
plt.title('XY Magnetometer Data')
plt.xlabel('X [uT]')
plt.ylabel('Y [uT]')
plt.legend()
plt.grid()
plt.axis('equal')

plt.figure()
plt.plot(rawData[:, 1], rawData[:, 2], 'b*', label='Raw Meas.')
plt.plot(calibData[:, 1], calibData[:, 2], 'r*', label='Calibrated Meas.')
plt.title('YZ Magnetometer Data')
plt.xlabel('Y [uT]')
plt.ylabel('Z [uT]')
plt.legend()
plt.grid()
plt.axis('equal')

plt.figure()
plt.plot(rawData[:, 0], rawData[:, 2], 'b*', label='Raw Meas.')
plt.plot(calibData[:, 0], calibData[:, 2], 'r*', label='Calibrated Meas.')
plt.title('XZ Magnetometer Data')
plt.xlabel('X [uT]')
plt.ylabel('Z [uT]')
plt.legend()
plt.grid()
plt.axis('equal')

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

for i in range(N):
	xraw = rawData[i, 0]
	yraw = rawData[i, 1]
	zraw = rawData[i, 2]
	
	xcalib = calibData[i, 0]
	ycalib = calibData[i, 1]
	zcalib = calibData[i, 2]
	
	ax.scatter(xraw, yraw, zraw, color='r')
	ax.scatter(xcalib, ycalib, zcalib, color='b')

ax.set_title('3D Scatter Plot of Magnetometer Data')
ax.set_xlabel('X [uT]')
ax.set_ylabel('Y [uT]')
ax.set_zlabel('Z [uT]')
'''
