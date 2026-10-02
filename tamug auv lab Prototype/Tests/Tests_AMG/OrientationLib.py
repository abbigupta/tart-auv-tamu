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
from ahrs.filters import Madgwick

import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtWidgets
from PyQt5.QtGui import QVector3D

sensor = adafruit_lsm9ds1.LSM9DS1_I2C(busio.I2C(board.SCL, board.SDA))

sys.path.append(os.path.abspath('../../Data/'))

# Setup QT Graph App
pg.mkQApp()
view = gl.GLViewWidget()
view.show()
view.setWindowTitle('AMG values')
view.setCameraPosition(distance=7.5)
view.resize(1920, 1080)

# Add Base XY Grid
grid = gl.GLGridItem(antialias=False)
grid.scale(1.0, 1.0, 1.0)
view.addItem(grid)

# Add Base Coordinate Axes
_x = gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [1, 0, 0]]), color=(1, 0, 0, 1), width=8, antialias=False)
_y = gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [0, 1, 0]]), color=(0, 1, 0, 1), width=8, antialias=False)
_z = gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [0, 0, 1]]), color=(0, 0, 1, 1), width=8, antialias=False)
view.addItem(_x)
view.addItem(_y)
view.addItem(_z)

# Add Object Coordinate Axes
_x2 = gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [1, 0, 0]]), color=(1, 0, 0, 1), width=4, antialias=False)
_y2 = gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [0, 1, 0]]), color=(0, 1, 0, 1), width=4, antialias=False)
_z2 = gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [0, 0, 1]]), color=(0, 0, 1, 1), width=4, antialias=False)
#view.addItem(_x2)
#view.addItem(_y2)
#view.addItem(_z2)

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

aScale = 1.0
mScale = 1.0
gScale = 1.0

# Add Measurement Vectors
vecAcc = gl.GLLinePlotItem(color=(1, 0, 0, 1), width=3, antialias=False)
vecMag = gl.GLLinePlotItem(color=(0, 1, 0, 1), width=3, antialias=False)
vecGyr = gl.GLLinePlotItem(color=(0, 0, 1, 1), width=3, antialias=False)
vecAcc.scale(aScale, aScale, aScale)
vecMag.scale(mScale, mScale, mScale)
vecGyr.scale(gScale, gScale, gScale)
view.addItem(vecAcc)
view.addItem(vecMag)
view.addItem(vecGyr)

# Add Measurement ScatterPlots
scatterAcc = gl.GLScatterPlotItem(glOptions='opaque')
scatterMag = gl.GLScatterPlotItem(glOptions='opaque')
scatterGyr = gl.GLScatterPlotItem(glOptions='opaque')
scatterAcc.scale(aScale, aScale, aScale)
scatterMag.scale(mScale, mScale, mScale)
scatterGyr.scale(gScale, gScale, gScale)
view.addItem(scatterAcc)
view.addItem(scatterMag)
view.addItem(scatterGyr)

# import accelerometer calibration
ab = np.zeros(3)
aA = np.identity(3)
fid = open('../../Data/aCalibration.txt','r')
for i in range(3):
	ab[i] = float(fid.readline())
for i in range(3):
	for j in range(3):
		aA[i][j] = float(fid.readline())
fid.close()

# import magnetometer calibration
mb = np.zeros(3)
mA = np.identity(3)
fid = open('mCalibration2.txt','r')
for i in range(3):
	mb[i] = float(fid.readline())
for i in range(3):
	for j in range(3):
		mA[i][j] = float(fid.readline())
fid.close()

# import gyroscope calibration
gb = np.zeros(3)
fid = open('../../Data/gCalibration.txt','r')
for i in range(3):
	gb[i] = float(fid.readline())
fid.close()

def aUseCalibration(a):
	return aA @ (a - ab)

def mUseCalibration(m):
	return mA @ (m - mb)

def gUseCalibration(g):
	return g - gb

# Lists of raw data
dataAccRaw = []
dataMagRaw = []
dataGyrRaw = []

# Lists of axes-converted data
dataAccAxes = []
dataMagAxes = []
dataGyrAxes = []

# Lists of calibrated data
dataAccCalib = []
dataMagCalib = []
dataGyrCalib = []
'''
ahrs = imufusion.Ahrs()
'''
madgwick = Madgwick(gain=1.0, Dt=0.05)
q = np.array([1.0, 0.0, 0.0, 0.0])


def update():
	
	global q
	
	# Collect data from sensor
	ax, ay, az = sensor.acceleration # Given in m/s^2
	mx, my, mz = sensor.magnetic     # Given in Gauss
	gx, gy, gz = sensor.gyro         # Given in rad/s
	
	# Set to numpy arrays
	a = np.array([ax, ay, az])
	m = np.array([mx, my, mz])
	g = np.array([gx, gy, gz])
	dataAccRaw.append(a)
	dataMagRaw.append(m)
	dataGyrRaw.append(g)
	
	# Convert sensor coordinates into proper axes from (NEU) to (ENU), mx was discovered to be sending flipped data
	a = np.array([ay, ax, az])
	m = np.array([my,-mx, mz])
	g = np.array([gy, gx, gz])
	dataAccAxes.append(a)
	dataMagAxes.append(m)
	dataGyrAxes.append(g)
	
	# Use calibration parameters
	a = aUseCalibration(a)
	m = mUseCalibration(m)
	g = gUseCalibration(g)
	dataAccCalib.append(a)
	dataMagCalib.append(m)
	dataGyrCalib.append(g)
	
	# Update measurement vectors and scatterplots with calibrated measurements
	scatterAcc.setData(pos=dataAccAxes, color=(1.0, 0.0, 0.0, 1.0), size=1)
	scatterMag.setData(pos=dataMagAxes, color=(0.0, 1.0, 0.0, 1.0), size=1)
	scatterGyr.setData(pos=dataGyrAxes, color=(0.0, 0.0, 1.0, 1.0), size=1)
	vecAcc.setData(pos=np.array([[0, 0, 0], dataAccAxes[-1] / np.linalg.norm(dataAccAxes[-1])]))
	vecMag.setData(pos=np.array([[0, 0, 0], dataMagAxes[-1] / np.linalg.norm(dataMagAxes[-1])]))
	vecGyr.setData(pos=np.array([[0, 0, 0], dataGyrAxes[-1]]))
	
	# Update imufusion with calibrated measurements
	q = madgwick.updateMARG(q=q, acc=a, mag=m, gyr=g)
	#q = madgwick.updateIMU(q=q, acc=a, gyr=g)
	rotMatrix = R.from_quat(np.array([q[1], q[2], q[3], q[0]])).as_matrix()
	yaw = math.atan2(-rotMatrix[1, 0], rotMatrix[1, 1])
	os.system('clear')
	print('Yaw: ',math.degrees(yaw))
	
	# Create transform for pyqtgraph
	T = np.identity(4)
	T[:3, :3] = rotMatrix
	Tr = pg.Transform3D(*T.flatten())
	
	# Apply transform to object coordinate axes and box
	_x2.setTransform(Tr)
	_y2.setTransform(Tr)
	_z2.setTransform(Tr)
	box.setTransform(Tr)
	

# Initiate pyqtgraph updater
timer = pg.QtCore.QTimer()
timer.timeout.connect(update)
timer.start(50)

# Open Window
QtWidgets.QApplication.instance().exec_()
