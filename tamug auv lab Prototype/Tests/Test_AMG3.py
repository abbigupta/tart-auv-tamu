import os
import sys
import time
import math
import numpy as np
import board
import busio
import adafruit_lsm9ds1
import imufusion
from scipy.spatial.transform import Rotation as R

import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtWidgets

sys.path.append(os.path.abspath('../Data/'))

sensor = adafruit_lsm9ds1.LSM9DS1_I2C(busio.I2C(board.SCL, board.SDA))

# Setup App
pg.mkQApp()
view = gl.GLViewWidget()
view.show()
view.setWindowTitle('3D Orientation')
view.setCameraPosition(distance=7.5)
#view.resize(1920, 1080)
view.resize(1024, 600)

# Add Grid
zgrid = gl.GLGridItem(antialias=False)
view.addItem(zgrid)
zgrid.scale(1.0, 1.0, 1.0)

# Add Axes
x_axis = gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [2, 0, 0]]), color=(1, 0, 0, 1), width=8, antialias=False, glOptions='opaque')
y_axis = gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [0, 2, 0]]), color=(0, 1, 0, 1), width=8, antialias=False, glOptions='opaque')
z_axis = gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [0, 0, 2]]), color=(0, 0, 1, 1), width=8, antialias=False, glOptions='opaque')
view.addItem(x_axis)
view.addItem(y_axis)
view.addItem(z_axis)

# Add Cube Axes
cx_axis = gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [2, 0, 0]]), color=(1, 0, 0, 1), width=8, antialias=False, glOptions='opaque')
cy_axis = gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [0, 2, 0]]), color=(0, 1, 0, 1), width=8, antialias=False, glOptions='opaque')
cz_axis = gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [0, 0, 2]]), color=(0, 0, 1, 1), width=8, antialias=False, glOptions='opaque')
view.addItem(cx_axis)
view.addItem(cy_axis)
view.addItem(cz_axis)

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

# import accelerometer calibration
ab = np.zeros(3)
'''
fid = open('Tests_AMG/AccelerometerCalibration.txt','r')
for i in range(3):
	ab[i] = float(fid.readline())
fid.close()
'''

# import magnetometer calibration
mb = np.zeros(3)
mA = np.zeros((3, 3))
fid = open('../Data/AMG_Calibration.txt','r')
for i in range(3):
	mb[i] = float(fid.readline())
for i in range(3):
	for j in range(3):
		mA[i][j] = float(fid.readline())
fid.close()

# import gyroscope calibration
gb = np.zeros(3)
fid = open('Tests_AMG/GyroCalibration.txt','r')
for i in range(3):
	gb[i] = float(fid.readline())
fid.close()

def AM2R(a, m):
	
	A = np.array([0, 0, 1])
	#M = np.array([0.01696934, 0.51762832, -0.85543729])
	M = np.array([0.17765338, 0.33174175, -0.9264916])
	
	p = np.cross(A-a, M-m)
	p /= np.linalg.norm(p)
	
	da = np.dot(np.cross(A, p), np.cross(a, p))/(np.linalg.norm(np.cross(A, p))*np.linalg.norm(np.cross(a, p)))
	dm = np.dot(np.cross(M, p), np.cross(m, p))/(np.linalg.norm(np.cross(M, p))*np.linalg.norm(np.cross(m, p)))
	
	w = (np.sign(np.dot(np.cross(a, A), p))*math.acos(da) + np.sign(np.dot(np.cross(m, M), p))*math.acos(dm))/2
	
	def RodRot(v, p, w):
		return v + math.sin(w)*np.cross(p, v) + (1-math.cos(w))*np.cross(p, np.cross(p, v))
	
	ux = np.array([1, 0, 0])
	uy = np.array([0, 1, 0])
	uz = np.array([0, 0, 1])
	
	rotMatrix = np.array([RodRot(ux, p, w), RodRot(uy, p, w), RodRot(uz, p, w)])
	
	return rotMatrix

def aCalibrate(a):
	return a - ab

def mCalibrate(m):
	return mA @ (m - mb)

def gCalibrate(g):
	return g - gb

ahrs = imufusion.Ahrs()

def update():
	
	# Collect data from sensor
	ax, ay, az = sensor.acceleration # Given in m/s^2
	mx, my, mz = sensor.magnetic     # Given in Gauss
	gx, gy, gz = sensor.gyro         # Given in rad/s
	
	# Convert sensor coordinates into proper axes (ENU)=(XYZ)
	a = np.array([ay, ax, az])
	m = np.array([my,-mx, mz]) # !! magnetometer gave it's raw x component flipped for some reason
	g = np.array([gy, gx, gz])
	
	# Gauss to microteslas
	m *= 100
	
	# Calibrate readings
	a = aCalibrate(a)
	m = mCalibrate(m)
	g = gCalibrate(g)
	
	# Normalize a and m for orientation calculation
	a /= np.linalg.norm(a)
	m /= np.linalg.norm(m)
	
	# Use a and m to find a rotation matrix from local coordinates to global coordinates
	rotMatrix = AM2R(a, m)
	# Or give imufusion a, m, and g to get a rotation matrix
	ahrs.update(a, m, g, 0.05)
	q = ahrs.quaternion
	rotMatrix2 = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
	
	# Create transform for pyqtgraph
	T = np.identity(4)
	T[:3, :3] = rotMatrix2
	boxTransform = pg.Transform3D(*T.T.flatten())
	
	# Apply transform to box
	box.setTransform(boxTransform)
	cx_axis.setTransform(boxTransform)
	cy_axis.setTransform(boxTransform)
	cz_axis.setTransform(boxTransform)

# Initiate pyqtgraph updater
timer = pg.QtCore.QTimer()
timer.timeout.connect(update)
timer.start(50)

# Open Window
QtWidgets.QApplication.instance().exec_()
