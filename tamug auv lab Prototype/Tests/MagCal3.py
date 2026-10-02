import os
import sys
import math
import numpy as np
import time
import board
import busio
import adafruit_lsm9ds1
from datetime import datetime

import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtWidgets
#from PyQt5.QtGui import QVector3D

#==================== INPUTS ===========================================
sample = 1600    #number of samples to take for calibration
location = 'shell_beach'
location = 'basin'
power = 0

#=======================================================================

if power >0: 
	sys.path.append(os.path.abspath('../Devices/'))
	import ESC
	esc=ESC.ESC([19, 20],turn_offset=0,speeding = power/100)
	esc.arm()
	esc.setMotionModel('Differential')
	print(f'Currently using {power}% of power')
	print('The motor will keep running until data samples are collected')
	
	motor_armed = False
	ans=input('Press Enter (or any key other than N) to continue:  ')
	if ans in ['N','n']:
		esc.disarm()
		sys.exit()
	else:
		esc.setMotion([1, 0.0])
		motor_armed = True
	

#=====================================================


# Setup sensor object
S1 = adafruit_lsm9ds1.LSM9DS1_I2C(busio.I2C(board.SCL, board.SDA))

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

# Add Measurement Vector & ScatterPlot
rescale = 1.0
vecMag = gl.GLLinePlotItem(color=(0, 1, 0, 1), width=3, antialias=False)
vecMag.scale(rescale, rescale, rescale)
view.addItem(vecMag)
scatterMag = gl.GLScatterPlotItem(glOptions='opaque')
scatterMag.scale(rescale, rescale, rescale)
view.addItem(scatterMag)
scatterMagCalib = gl.GLScatterPlotItem(glOptions='opaque')
scatterMagCalib.scale(rescale, rescale, rescale)
view.addItem(scatterMagCalib)

# Import magnetometer calibration
mb = np.zeros(3)
mA = np.identity(3)
fid = open('mCalibration2.txt','r')
for i in range(3):
	mb[i] = float(fid.readline())
for i in range(3):
	for j in range(3):
		mA[i][j] = float(fid.readline())
fid.close()

# Function for calibrating a measurement
def mUseCalibration(m):
	return mA @ (m - mb)

# Lists of data
dataMagRaw = []
dataMagAxes = []
dataMagCalib = []

n = 0
Kb = 1.0
KA = 1.0

# define parameters to change (13)
bx = 0.0
by = 0.0
bz = 0.0
r = 1.0
Ax1 = 1.0
Ax2 = 0.0
Ax3 = 0.0
Ay1 = 0.0
Ay2 = 1.0
Ay3 = 0.0
Az1 = 0.0
Az2 = 0.0
Az3 = 1.0

complete = False


def update():
	
	global bx, by, bz, r, n, nc, Kb, KA, dataMagRaw, dataMagAxes, dataMagCalib, complete,sample,motor_armed
	global Ax1, Ax2, Ax3, Ay1, Ay2, Ay3, Az1, Az2, Az3
	
	'''
	---------------------------------
	Collect c Data Points
	---------------------------------
	'''
	if n < sample:
		
		# Collect data from sensor (Given in Gauss)
		mx, my, mz = S1.magnetic
		
		# Set to numpy array
		m = np.array([mx, my, mz])
		dataMagRaw.append(m)
		
		# Convert sensor coordinates into proper axes from (NEU) to (ENU), mx was discovered to be sending flipped data
		m = np.array([my,-mx, mz])
		dataMagAxes.append(m)
		
		# Update Measurement Vector and ScatterPlot
		scatterMag.setData(pos=dataMagAxes, color=(0.0, 1.0, 0.0, 1.0), size=1)
		vecMag.setData(pos=np.array([[0, 0, 0], dataMagAxes[-1]]))
	else:
		if power>0 and motor_armed:
			esc.disarm()
			motor_armed = False
	
	'''
	---------------------------------
	Use gradient descent to center the data onto the origin
	
	Finding a 3D point (b) and radius (r) that has the least sum of square distances minus radius to all of the data points
	
	Value to minimize: V = SUM((distance(b, data_i) - r)^2)
	---------------------------------
	'''
	if n >= sample and Kb > 0.0000000001:
		
		# define value to minimize
		def V(vbx, vby, vbz, vr):
			dMA = np.array(dataMagAxes)
			sum_squares = 0.0
			for i in range(len(dMA)):
				sum_squares += (math.sqrt((dMA[i,0] - vbx)**2 + (dMA[i,1] - vby)**2 + (dMA[i,2] - vbz)**2) - vr)**2
			return sum_squares
		
		# define slope estimates from stepping into each parameter
		sbx = (V(bx + 0.001, by, bz, r) - V(bx, by, bz, r)) / 0.01 / sample
		sby = (V(bx, by + 0.001, bz, r) - V(bx, by, bz, r)) / 0.01 / sample
		sbz = (V(bx, by, bz + 0.001, r) - V(bx, by, bz, r)) / 0.01 / sample
		sr = (V(bx, by, bz, r + 0.001) - V(bx, by, bz, r)) / 0.01 / sample
		
		# step parameters based on slopes with a clamp of 0.01
		bx -= min(max(sbx, -0.01), 0.01)
		by -= min(max(sby, -0.01), 0.01)
		bz -= min(max(sbz, -0.01), 0.01)
		r -= min(max(sr, -0.01), 0.01)
		
		# prepare A before tuning
		Ax1 = 1/r
		Ay2 = 1/r
		Az3 = 1/r
		
		# continually print sum of squares of slopes to see how small it becomes
		os.system('clear')
		print(f'bx: {bx}')
		print(f'by: {by}')
		print(f'bz: {bz}')
		print(f'r: {r}')
		Kb = (sbx)**2 + (sby)**2 + (sbz)**2 + (sr)**2
		print(f'closeness: {Kb}')
		
		# Update Calibrated ScatterPlot
		dataMagCalib = np.array(dataMagAxes)
		for i in range(len(dataMagCalib)):
			dataMagCalib[i] = dataMagAxes[i] - np.array([bx, by, bz])
		#scatterMagCalib.setData(pos=dataMagCalib, color=(1.0, 1.0, 0.0, 1.0), size=1)
	
	'''
	---------------------------------
	Use gradient descent to correct for warping in the data
	
	Finding a 3x3 Matrix (A) that makes the data as close as possible to a unit sphere
	
	Value to minimize: V = SUM((magnitude(A*(data_i - b)) - 1.0)^2)
	---------------------------------
	'''
	if n >= sample and Kb <= 0.0000000001 and KA > 0.00000001:
		
		# define value to minimize
		def V(vAx1, vAx2, vAx3, vAy1, vAy2, vAy3, vAz1, vAz2, vAz3):
			dMA = np.array(dataMagAxes)
			sum_squares = 0.0
			A = np.array([[vAx1, vAx2, vAx3], [vAy1, vAy2, vAy3], [vAz1, vAz2, vAz3]])
			for i in range(len(dMA)):
				dMAi = A @ (dMA[i] - np.array([bx, by, bz]))
				sum_squares += (np.linalg.norm(dMAi) - 1.0)**2
			return sum_squares
		
		# define slope estimates from stepping into each parameter
		sAx1 = (V(Ax1 + 0.001, Ax2, Ax3, Ay1, Ay2, Ay3, Az1, Az2, Az3) - V(Ax1, Ax2, Ax3, Ay1, Ay2, Ay3, Az1, Az2, Az3)) / 0.01 / sample * 3
		sAx2 = (V(Ax1, Ax2 + 0.001, Ax3, Ay1, Ay2, Ay3, Az1, Az2, Az3) - V(Ax1, Ax2, Ax3, Ay1, Ay2, Ay3, Az1, Az2, Az3)) / 0.01 / sample * 3
		sAx3 = (V(Ax1, Ax2, Ax3 + 0.001, Ay1, Ay2, Ay3, Az1, Az2, Az3) - V(Ax1, Ax2, Ax3, Ay1, Ay2, Ay3, Az1, Az2, Az3)) / 0.01 / sample * 3
		sAy1 = (V(Ax1, Ax2, Ax3, Ay1 + 0.001, Ay2, Ay3, Az1, Az2, Az3) - V(Ax1, Ax2, Ax3, Ay1, Ay2, Ay3, Az1, Az2, Az3)) / 0.01 / sample * 3
		sAy2 = (V(Ax1, Ax2, Ax3, Ay1, Ay2 + 0.001, Ay3, Az1, Az2, Az3) - V(Ax1, Ax2, Ax3, Ay1, Ay2, Ay3, Az1, Az2, Az3)) / 0.01 / sample * 3
		sAy3 = (V(Ax1, Ax2, Ax3, Ay1, Ay2, Ay3 + 0.001, Az1, Az2, Az3) - V(Ax1, Ax2, Ax3, Ay1, Ay2, Ay3, Az1, Az2, Az3)) / 0.01 / sample * 3
		sAz1 = (V(Ax1, Ax2, Ax3, Ay1, Ay2, Ay3, Az1 + 0.001, Az2, Az3) - V(Ax1, Ax2, Ax3, Ay1, Ay2, Ay3, Az1, Az2, Az3)) / 0.01 / sample * 3
		sAz2 = (V(Ax1, Ax2, Ax3, Ay1, Ay2, Ay3, Az1, Az2 + 0.001, Az3) - V(Ax1, Ax2, Ax3, Ay1, Ay2, Ay3, Az1, Az2, Az3)) / 0.01 / sample * 3
		sAz3 = (V(Ax1, Ax2, Ax3, Ay1, Ay2, Ay3, Az1, Az2, Az3 + 0.001) - V(Ax1, Ax2, Ax3, Ay1, Ay2, Ay3, Az1, Az2, Az3)) / 0.01 / sample * 3
		
		# step parameters based on slopes with a clamp of 0.001
		Ax1 -= min(max(sAx1, -0.001), 0.001)
		Ax2 -= min(max(sAx2, -0.001), 0.001)
		Ax3 -= min(max(sAx3, -0.001), 0.001)
		Ay1 -= min(max(sAy1, -0.001), 0.001)
		Ay2 -= min(max(sAy2, -0.001), 0.001)
		Ay3 -= min(max(sAy3, -0.001), 0.001)
		Az1 -= min(max(sAz1, -0.001), 0.001)
		Az2 -= min(max(sAz2, -0.001), 0.001)
		Az3 -= min(max(sAz3, -0.001), 0.001)
		
		# continually print sum of squares of slopes to see how small it becomes
		os.system('clear')
		print(f'Ax1: {Ax1}')
		print(f'Ax2: {Ax2}')
		print(f'Ax3: {Ax3}')
		print(f'Ay1: {Ay1}')
		print(f'Ay2: {Ay2}')
		print(f'Ay3: {Ay3}')
		print(f'Az1: {Az1}')
		print(f'Az2: {Az2}')
		print(f'Az3: {Az3}')
		KA = (sAx1)**2 + (sAx2)**2 + (sAx3)**2 + (sAy1)**2 + (sAy2)**2 + (sAy3)**2 + (sAz1)**2 + (sAz2)**2 + (sAz3)**2
		print(f'closeness: {KA}')
		
		# Update Calibrated ScatterPlot
		dataMagCalib = np.array(dataMagAxes)
		for i in range(len(dataMagCalib)):
			dataMagCalib[i] = np.array([[Ax1, Ax2, Ax3], [Ay1, Ay2, Ay3], [Az1, Az2, Az3]]) @ dataMagAxes[i] - np.array([bx, by, bz])
		scatterMagCalib.setData(pos=dataMagCalib, color=(1.0, 1.0, 0.0, 1.0), size=1)
	
	if n >= sample and KA <= 0.00000001 and not complete:
		# Save mCalibration to file
		timestamp = datetime.now().strftime('%Y%m%d%H%M')
		mCalibration = open(f'../Data/mCalibration2_{location}_{power}_{timestamp}.txt','w')
		mCalibration.write(f'{bx}\n')
		mCalibration.write(f'{by}\n')
		mCalibration.write(f'{bz}\n')
		mCalibration.write(f'{Ax1}\n')
		mCalibration.write(f'{Ax2}\n')
		mCalibration.write(f'{Ax3}\n')
		mCalibration.write(f'{Ay1}\n')
		mCalibration.write(f'{Ay2}\n')
		mCalibration.write(f'{Ay3}\n')
		mCalibration.write(f'{Az1}\n')
		mCalibration.write(f'{Az2}\n')
		mCalibration.write(f'{Az3}\n')
		mCalibration.close()
		os.system(f'cd ../Data/; ln -sf mCalibration2_{location}_{power}_{timestamp}.txt mCalibration.txt')
		
		complete = True
		
	
	n += 1

# Initiate pyqtgraph updater
timer = pg.QtCore.QTimer()
timer.timeout.connect(update)
timer.start(50)

# Open Window
QtWidgets.QApplication.instance().exec_()
