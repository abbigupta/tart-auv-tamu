import os
import sys
import math
import numpy as np
import time
import board
import busio
import adafruit_lsm9ds1
import matplotlib.pyplot as plt
import imufusion
from scipy.spatial.transform import Rotation as R

import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtWidgets
from PyQt5.QtGui import QVector3D

sensor = adafruit_lsm9ds1.LSM9DS1_I2C(busio.I2C(board.SCL, board.SDA))

sys.path.append(os.path.abspath('../../Data/'))

# Setup App
pg.mkQApp()
view = gl.GLViewWidget()
view.show()
view.setWindowTitle('AMG values')
view.setCameraPosition(distance=7.5)
view.resize(1920, 1080)
#view.resize(1024, 600)

# Add Grid
zgrid = gl.GLGridItem(antialias=False)
view.addItem(zgrid)
zgrid.scale(1.0, 1.0, 1.0)

# Add Axes
x_axis = gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [1, 0, 0]]), color=(1, 0, 0, 1), width=8, antialias=False)
y_axis = gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [0, 1, 0]]), color=(0, 1, 0, 1), width=8, antialias=False)
z_axis = gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [0, 0, 1]]), color=(0, 0, 1, 1), width=8, antialias=False)
view.addItem(x_axis)
view.addItem(y_axis)
view.addItem(z_axis)

# Add Object Axes
x_axisb = gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [2, 0, 0]]), color=(1, 0, 0, 1), width=4, antialias=False)
y_axisb = gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [0, 2, 0]]), color=(0, 1, 0, 1), width=4, antialias=False)
z_axisb = gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [0, 0, 2]]), color=(0, 0, 1, 1), width=4, antialias=False)
view.addItem(x_axisb)
view.addItem(y_axisb)
view.addItem(z_axisb)

# Add Angle-Axis Vector
angleAxis = gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [1, 0, 0]]), color=(1, 1, 1, 1), width=3, antialias=False)
#view.addItem(angleAxis)

# Create a visible cube using GLMeshItem
lx = 7.0/5
ly = 2.0/5
lz = 0.3/5
verts = np.array([
    [-lx, -ly, -lz],
    [ lx, -ly, -lz],
    [ lx,  ly, -lz],
    [-lx,  ly, -lz],
    [-lx, -ly,  lz],
    [ lx, -ly,  lz],
    [ lx,  ly,  lz],
    [-lx,  ly,  lz],
])

faces = np.array([
    [0,1,2], [0,2,3],
    [1,5,6], [1,6,2],
    [5,4,7], [5,7,6],
    [4,0,3], [4,3,7],
    [3,2,6], [3,6,7],
    [1,0,4], [1,4,5]
])

colors = np.array([
    [1.0, 0.0, 0.0, 1.0],  # red
    [1.0, 0.5, 0.0, 1.0],  # orange
    [1.0, 1.0, 0.0, 1.0],  # yellow
    [0.0, 1.0, 0.0, 1.0],  # green
    [0.0, 1.0, 1.0, 1.0],  # cyan
    [0.0, 0.0, 1.0, 1.0],  # blue
    [0.5, 0.0, 1.0, 1.0],  # purple
    [1.0, 0.0, 1.0, 1.0],  # magenta
    [0.6, 0.6, 0.6, 1.0],  # gray
    [0.3, 0.3, 0.3, 1.0],  # dark gray
    [0.8, 0.4, 0.0, 1.0],  # brown
    [0.2, 0.8, 0.2, 1.0],  # lime green
])

box = gl.GLMeshItem(vertexes=verts, faces=faces, faceColors=colors, smooth=False, drawEdges=True)
view.addItem(box)

# Add Display Vectors
vecAcc = gl.GLLinePlotItem(color=(1, 0, 0, 1), width=3, antialias=False)
vecMag = gl.GLLinePlotItem(color=(0, 1, 0, 1), width=3, antialias=False)
vecGyr = gl.GLLinePlotItem(color=(0, 0, 1, 1), width=3, antialias=False)
#view.addItem(vecAcc)
#view.addItem(vecMag)
#view.addItem(vecGyr)

# Add ScatterPlots
scatterAcc = gl.GLScatterPlotItem(glOptions='opaque')
scatterMag = gl.GLScatterPlotItem(glOptions='opaque')
scatterGyr = gl.GLScatterPlotItem(glOptions='opaque')
#view.addItem(scatterAcc)
#view.addItem(scatterMag)
#view.addItem(scatterGyr)
plotAcc = []
plotMag = []
plotGyr = []

# import accelerometer calibration
ab = np.zeros(3)
#fid = open('AccelerometerCalibration.txt','r')
#for i in range(3):
#	ab[i] = float(fid.readline())
#fid.close()

# import magnetometer calibration
mb = np.zeros(3)
mA = np.zeros((3, 3))
fid = open('../../Data/AMG_Calibration.txt','r')
for i in range(3):
	mb[i] = float(fid.readline())
for i in range(3):
	for j in range(3):
		mA[i][j] = float(fid.readline())
fid.close()

# import gyroscope calibration
gb = np.zeros(3)
fid = open('GyroCalibration.txt','r')
for i in range(3):
	gb[i] = float(fid.readline())
fid.close()

def mCalibrateCreate():
	N = 100
	D = np.array(dataMag)
	DC = np.array(dataMag)
	
	b = np.mean(D, axis=0)
	s = np.array([1,1,1])
	A = np.identity(3, dtype=float)
	S = np.identity(3, dtype=float)
	
	def v(p):
		return np.std(np.linalg.norm(b-D, axis=1)) - np.std(np.linalg.norm(p-D, axis=1))
	
	def V(M):
		return np.mean((np.linalg.norm(A @ DC.T, axis=0) - R)**2) - np.mean((np.linalg.norm((A+M) @ DC.T, axis=0) - R)**2)
	V2 = np.vectorize(V, signature='(m,n)->()')
	
	for _ in range(500):
		s = np.array(np.apply_along_axis(v, axis=1, arr = b+np.identity(3)*0.001)) * 1000
		b += s
	DC = D - b
	R = np.linalg.norm(DC, axis=1)
	
	I = np.zeros((3,3,3,3), dtype=float)
	for i in range(9):
		I[i//3, i%3, i//3, i%3] = 0.001
	for _ in range(500):
		S = V2(I)
		A += S
	
	# Save calibration in file
	cData = open('../../Data/AMG_Calibration.txt','w')
	for f in b:
		cData.write(str(f)+'\n')
	for r in A:
		for f in r:
			cData.write(str(f)+'\n')
	cData.close()

Z = np.array([0.0,0.0,0.01])
rotMatrixPrev = np.identity(3)

def AM2R(a, m, g, dt):
	
	global up, north, Z, rotMatrixPrev
	
	gGlobal = rotMatrixPrev.T @ g
	'''
	Zw = math.acos((np.trace(rotMatrixPrev)-1)/2)
	if (Zw == 0.0): Zw += 0.0001
	Zp = np.array([rotMatrixPrev[2][1] - rotMatrixPrev[1][2], rotMatrixPrev[0][2] - rotMatrixPrev[2][0], rotMatrixPrev[1][0] - rotMatrixPrev[0][1]])/(2*math.sin(Zw))
	Z = -Zp*Zw
	if (np.linalg.norm(Z) == 0.0): Z = np.array([0.0,0.0,1.0])
	'''
	ux = np.array([1, 0, 0])
	uy = np.array([0, 1, 0])
	uz = np.array([0, 0, 1])
	
	A = up
	M = north
	
	def RodRot(v, p, w):
		return v + math.sin(w)*np.cross(p, v) + (1-math.cos(w))*np.cross(p, np.cross(p, v))
	
	def V(z):
		zm = np.linalg.norm(z)
		av2 = np.linalg.norm(RodRot(a, z/zm, zm) - A)**2
		mv2 = np.linalg.norm(RodRot(m, z/zm, zm) - M)**2
		return av2 + mv2
	
	def mat(p, w):
		return np.array([RodRot(ux, p, w), RodRot(uy, p, w), RodRot(uz, p, w)])
	
	def matg(M, p, w):
		return np.array([RodRot(M[0], p, w*dt), RodRot(M[1], p, w*dt), RodRot(M[2], p, w*dt)])
	
	rotMatrixG = matg(rotMatrixPrev, gGlobal/np.linalg.norm(gGlobal), np.linalg.norm(gGlobal))
	
	I = np.identity(3)/10000
	
	for i in range(10):
		Zstep = V(Z)*np.ones(3) - np.array([V(Z+I[0]), V(Z+I[1]), V(Z+I[2])])
		Z = Z + 900 * Zstep
	
	Zm = np.linalg.norm(Z)
	cycleReturn = Zm**(np.floor(Zm/(2*math.pi)))
	Z /= cycleReturn
	Zm = np.linalg.norm(Z)
	
	rotMatrixAM = mat(Z/Zm, Zm)
	
	angleAxis.setData(pos=np.array([[0, 0, 0], Z]))
	'''
	# Gyro complimentary filter
	if (cycleReturn == 1):
		alpha = 0.25
		rotMatrixPrev = alpha*rotMatrixAM + (1.0-alpha)*rotMatrixG
	else:
		rotMatrixPrev = rotMatrixAM
	'''
	'''
	# Standard complimentary filter
	if (abs(cycleReturn-1.0) < 0.0001):
		alpha = 0.5
		rotMatrixPrev = alpha*rotMatrixAM + (1.0-alpha)*rotMatrixPrev
	else:
		print('cycleReturn')
		rotMatrixPrev = rotMatrixAM
	'''
	# No filter
	rotMatrixPrev = rotMatrixAM
	
	return rotMatrixPrev

def aCalibrate(a):
	return a - ab

def mCalibrate(m):
	return mA @ (m - mb)

def gCalibrate(g):
	return g - np.array([gb[1], gb[0], gb[2]])

# Lists of raw data
dataAcc = []
dataMag = []
dataGyr = []

calibrated = True

# Initial reference vectors for orientation
up = np.zeros(3)
north = np.zeros(3)

ahrs = imufusion.Ahrs()

lastTime = 0

def update():
	
	global calibrated, up, north, lastTime
	
	initPeriod = 100
	
	# Collect data from sensor
	ax, ay, az = sensor.acceleration # Given in m/s^2
	mx, my, mz = sensor.magnetic     # Given in Gauss
	gx, gy, gz = sensor.gyro         # Given in rad/s
	
	# Convert sensor coordinates into proper axes (ENU)/(XYZ)
	a = np.array([ay, ax, az])
	m = np.array([my,-mx, mz])
	g = np.array([gy, gx, gz])
	
	# Gauss to microteslas
	m *= 100
	g *= 180/math.pi
	
	# Add to uncalibrated data lists
	dataAcc.append(a)
	dataMag.append(m)
	dataGyr.append(g)
	
	# Calibrate readings
	a = aCalibrate(a)
	m = mCalibrate(m)
	g = gCalibrate(g)
	
	# Add to plot lists
	plotAcc.append(a/5)
	plotMag.append(m/25)
	plotGyr.append(g)
	
	if (len(dataAcc) > initPeriod):
		# Normalize a and m for orientation calculation
		a /= np.linalg.norm(a)
		m /= np.linalg.norm(m)
		
		# My own filter seems to work better
		dt = time.time() - lastTime
		lastTime = time.time()
		'''
		rotMatrix = AM2R(a, m, g, dt)
		'''
		# imufusion (didn't work correctly?)
		ahrs.update(a, m, g, dt)
		q = ahrs.quaternion
		rotMatrix = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
		
		
		# Create transform for pyqtgraph
		T = np.identity(4)
		T[:3, :3] = rotMatrix
		Tr = pg.Transform3D(*T.T.flatten())
		
		scatterAcc.setData(pos=plotAcc, color=(1.0, 0.0, 0.0, 1.0), size=1)
		scatterMag.setData(pos=plotMag, color=(0.0, 1.0, 0.0, 1.0), size=1)
		scatterGyr.setData(pos=plotGyr, color=(0.0, 0.0, 1.0, 1.0), size=1)
		
		vecAcc.setData(pos=np.array([[0, 0, 0], plotAcc[-1]]))
		vecMag.setData(pos=np.array([[0, 0, 0], plotMag[-1]]))
		vecGyr.setData(pos=np.array([[0, 0, 0], plotGyr[-1]]))
		
		#print(np.dot(a, m))
		
		x_axisb.setTransform(Tr)
		y_axisb.setTransform(Tr)
		z_axisb.setTransform(Tr)
		'''
		vecAcc.setTransform(Tr)
		vecMag.setTransform(Tr)
		vecGyr.setTransform(Tr)
		'''
		box.setTransform(Tr)
	
	if ((not calibrated) and (len(dataAcc) == initPeriod)):
		calibrated = True
		mCalibrateCreate()
	
	if (len(dataAcc) == initPeriod):
		up = np.mean(dataAcc, axis=0)
		up /= np.linalg.norm(up)
		north = np.mean(dataMag, axis=0)
		north /= np.linalg.norm(north)
		print('up: ',up)
		print('north: ',north)

# Initiate pyqtgraph updater
timer = pg.QtCore.QTimer()
timer.timeout.connect(update)
timer.start(25)

# Open Window
QtWidgets.QApplication.instance().exec_()
