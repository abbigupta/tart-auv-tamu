import os
import sys
import math
import time
import board
import busio
import statistics
import numpy as np
import adafruit_lsm9ds1
#from ahrs.filters import Madgwick
from scipy.spatial.transform import Rotation as sciR
from scipy.interpolate import interp1d
from functions import diff_yaw,mean_yaw,std_yaw,calculate_yaw
from collections import deque

sys.path.append(os.path.abspath('../Data/'))

class AMG:
	
	# Library access
	S1 = adafruit_lsm9ds1.LSM9DS1_I2C(busio.I2C(board.SCL, board.SDA))
	
	# Acceleration, Magnetic Field, and Angular Velocity vectors
	a = np.zeros(3)
	m = np.zeros(3)
	g = np.zeros(3)
	
	# Raw data
	aData = []
	mData = []
	gData = []
	
	# Import sensor calibrations
	ab = np.zeros(3)
	aA = np.identity(3)
	mb = np.zeros(3)
	mA = np.identity(3)
	gb = np.zeros(3)
	try:
		afid = open('../Data/aCalibration.txt','r')
		mfid = open('../Data/mCalibration.txt','r')
		gfid = open('../Data/gCalibration.txt','r')
		for i in range(3):
			ab[i] = float(afid.readline())
			mb[i] = float(mfid.readline())
			gb[i] = float(gfid.readline())
		for i in range(3):
			for j in range(3):
				aA[i][j] = float(afid.readline())
				mA[i][j] = float(mfid.readline())
		afid.close()
		mfid.close()
		gfid.close()
	except:
		pass
	
	# Orientation Variables
	q = np.array([1.0, 0.0, 0.0, 0.0])
	#madgwick = Madgwick(gain=0.042, Dt=0.05)  # Placeholder for Madgwick filter instance
	Z = np.array([0.0, 0.0, 0.01])
	R = np.identity(3)
	u_ = np.zeros(3)
	n_ = np.zeros(3)
	
	# Euler angles (ZYX order)
	roll = 0.0
	pitch = 0.0
	yaw = 0.0
	offset = 0.0
	yaws = deque([], maxlen=500) #yaw for each second in the past
	itick = 0
	tps = 20 #tick per second
	ntick_avg = 1 # for yaw reading, how many ticks to avg; using 20 (same as tps) means avg over 1 second; used when calling getYaw() 
	
	status = 'Initializing'
	flag_aCalibrate = False
	flag_mCalibrate = False
	flag_gCalibrate = False
	count_aCalibrate = 0
	count_mCalibrate = 0
	count_gCalibrate = 0
	pitch_correction = 1 #0 for no pitch correction; 1 for using acc_pitch
	manual_correction = False
	lm = None #linear model
	method = 2 #1 for without any calibration; use 2
	
	def __init__(self, flag_aCalibrate = False, flag_mCalibrate = False, flag_gCalibrate = False,
	 offset = 0, pitch_correction = 1, manual_correction = False, ntick_avg = 1):
		self.flag_aCalibrate = flag_aCalibrate
		self.flag_mCalibrate = flag_mCalibrate
		self.flag_gCalibrate = flag_gCalibrate
		self.offset = offset
		self.status = 'Initialized'
		self.pitch_correction = pitch_correction
		self.manual_correction = manual_correction
		self.ntick_avg = ntick_avg
		
		if self.manual_correction:
			data = np.loadtxt('../Data/manual_correction.csv',delimiter=',')
			self.lm = interp1d(data[:,0],data[:,1],kind='linear')
	
	def printRawData(self):
		print(f"RawAccelX: {self.a[0]:.2f}")
		print(f"RawAccelY: {self.a[1]:.2f}")
		print(f"RawAccelZ: {self.a[2]:.2f}")
		print(f"RawMagX: {self.m[0]:.2f}")
		print(f"RawMagY: {self.m[1]:.2f}")
		print(f"RawMagZ: {self.m[2]:.2f}")
		print(f"RawRotVelX: {self.g[0]:.2f}")
		print(f"RawRotVelY: {self.g[1]:.2f}")
		print(f"RawRotVelZ: {self.g[2]:.2f}")
	
	def printData(self):
		#print('\n---AMG---------------------')
		#print(self.status)
		print(f'[AMG] Yaw: {self.yaw%360:.2f}; Offset: {self.offset:.0f}; Corrected yaw: {self.getYaw():.0f}')
		print(f'[AMG] Pitch (deg, negative for uptilt): {self.pitch:.2f}; method: {self.method}')
		print(f"[AMG] RawAccelX, Y, Z: {self.a[0]:.3f}, {self.a[1]:.3f}, {self.a[2]:.3f}")
		print(f"[AMG] RawMagX: {self.m[0]:.3f}, {self.m[1]:.3f}, {self.m[2]:.3f}")
		print(f'[AMG] ntick_avg:',self.ntick_avg)
		#if len(self.yaws)>200: print(f'std of last 200 yaws: {std_yaw(np.array(self.yaws)[0:200]):.0f}')
		
	
	def getRoll(self):
		return self.roll
	
	def getPitch(self):
		return self.pitch
	
	def getYaw(self):
		tyaw = self.yaw
		if self.ntick_avg > 1 and len(self.yaws)>=self.ntick_avg: #use mean
			tyaw = mean_yaw(np.array(self.yaws)[0:self.ntick_avg])
		if self.manual_correction:
			tyaw = self.lm(tyaw%360)
		return (tyaw + self.offset)%360
	
	def tick(self):
		self.itick += 1
		# Collect data from sensor
		ax, ay, az = self.S1.acceleration # Given in m/s^2
		mx, my, mz = self.S1.magnetic     # Given in Gauss
		gx, gy, gz = self.S1.gyro         # Given in rad/s
		
		# Convert sensor coordinates into proper axes (ENU)/(XYZ)
		self.a = np.array([ay, ax, az])
		self.m = np.array([my,-mx, mz])
		self.g = np.array([gy, gx, gz])
		
		# Unit conversions
		#self.m *= 100
		#self.g *= 180/math.pi
		
		# Append data to lists
		self.aData.append(self.a)
		self.mData.append(self.m)
		self.gData.append(self.g)
		
		# Accelerometer Calibration sequence
		if self.flag_aCalibrate:
			if (self.count_aCalibrate == 0):
				for i in range(5):
					self.status = f'Prepare for Accelerometer Calibration ({5-i}s)\nInstructions: Slowly rotate device in all orientations - avoid jerky motion'
					time.sleep(1)
			self.count_aCalibrate += 1
			self.status = f'Calibration: Collecting data for Accelerometer {self.count_aCalibrate}/500\nInstructions: Slowly rotate device in all orientations - avoid jerky motions'
			if (self.count_aCalibrate == 500):
				self.flag_aCalibrate = False
				self.aCalibrateCreate()
		
		# Magnetometer Calibration sequence
		elif self.flag_mCalibrate:
			if (self.count_mCalibrate == 0):
				for i in range(5):
					self.status = f'Prepare for Magnetometer Calibration ({5-i}s)\nInstructions: Slowly rotate device in all orientations - avoid magnetic fields'
					time.sleep(1)
			self.count_mCalibrate += 1
			self.status = f'Calibration: Collecting data for Magnetometer {self.count_mCalibrate}/500\nInstructions: Slowly rotate device in all orientations - avoid magnetic fields'
			if (self.count_mCalibrate == 500):
				self.flag_mCalibrate = False
				self.mCalibrateCreate()
		
		# Gyroscope Calibration sequence
		elif self.flag_gCalibrate:
			if (self.count_gCalibrate == 0):
				for i in range(1):
					self.status = f'Prepare for Gyroscope Calibration ({5-i}s)\n Instructions: Lay device flat on a stable surface'
					time.sleep(1)
			self.count_gCalibrate += 1
			self.status = f'Calibration: Collecting data for Gyroscope {self.count_gCalibrate}/100\nInstructions: Lay device flat on a stable surface'
			if (self.count_gCalibrate == 100):
				self.flag_gCalibrate = False
				self.gCalibrateCreate()
				self.u_ = np.mean(np.array(self.aData[-50:]), axis=0)
				self.u_ /= np.linalg.norm(self.u_)
				self.n_ = np.mean(np.array(self.mData[-50:]), axis=0)
				self.n_ /= np.linalg.norm(self.n_)
		
		# Standard Protocol
		elif self.method == 1: #without any calibration
			a,m,g=self.a,self.m,self.g
			self.yaw = math.degrees(math.atan2(-m[0],m[1]))
			if self.pitch_correction == 1:
				ax,ay,az = a
				my=m[0]; mz=m[2] #see line136;
				mx=m[1];  #change mx=m[1] if the pitch correction goes weird
				pitch = -1*math.atan2(ay, math.sqrt(ax**2 + az**2)) #get pich from acc; will be noisy as robot acc or deacc, but stable when avg over time
				mx_comp = mx
				my_comp = my * math.cos(pitch) - mz * math.sin(pitch)
				self.yaw = math.degrees(math.atan2(-my_comp, mx_comp))
				self.pitch = math.degrees(pitch)
			self.yaws.appendleft(self.yaw)
			return 
		elif self.method==2:
			a = self.aCalibrate(self.a)
			m = self.mCalibrate(self.m)
			g = self.gCalibrate(self.g)  #g is not used
			
			a /= np.linalg.norm(a)
			m /= np.linalg.norm(m)
			# simple way to get yaw 
			self.yaw = math.degrees(math.atan2(-m[0],m[1]))
			if self.pitch_correction == 1:
				ax,ay,az = a
				my=m[0]; mz=m[2] #see line136;
				mx=m[1];  #change mx=m[1] if the pitch correction goes weird
				#pitch = -1*math.atan2(ay, math.sqrt(ax**2 + az**2)) #when y is facing forward 
				pitch = -1*math.atan2(-ax, math.sqrt(ay*ay + az*az)) #get pich from acc; will be noisy as robot acc or deacc, but stable when avg over time
				mx_comp = mx   #mx stay unchanged, as pitch rotate on x direction
				my_comp = my * math.cos(pitch) - mz * math.sin(pitch)
				self.yaw = math.degrees(math.atan2(-my_comp, mx_comp))
				self.pitch = math.degrees(pitch)
			self.yaws.appendleft(self.yaw)

			return

			'''
			# make accleration and gyro to zero
			self.Orientation(a, m, g)
			#self.yaw = math.degrees(math.atan2(self.R[0][1], self.R[0][0]))
			self.pitch = -math.degrees(math.atan2(self.R[0][2], math.hypot(self.R[0][0], self.R[0][1])))
			self.status = f'Active'
			'''
	
	def aCalibrate(self, a):
		return self.aA @ (a - self.ab)

	def mCalibrate(self, m):
		return self.mA @ (m - self.mb)

	def gCalibrate(self, g):
		return g - self.gb
	
	def aCalibrateCreate(self):
		D = np.array(self.aData[-500:])
		b = np.mean(D, axis=0)
		s = np.array([1,1,1])
		A = np.identity(3, dtype=float)
		S = np.identity(3, dtype=float)
		
		def v(p):
			return np.std(np.linalg.norm(b-D, axis=1)) - np.std(np.linalg.norm(p-D, axis=1))
		
		def V(M):
			return np.mean((np.linalg.norm(A @ D.T, axis=0) - R)**2) - np.mean((np.linalg.norm((A+M) @ D.T, axis=0) - R)**2)
		V2 = np.vectorize(V, signature='(m,n)->()')
		
		for _ in range(500):
			s = np.array(np.apply_along_axis(v, axis=1, arr = b+np.identity(3)*0.001)) * 1000
			b += s
		D -= b
		R = np.linalg.norm(D, axis=1)
		
		I = np.zeros((3,3,3,3), dtype=float)
		for i in range(9):
			I[i//3, i%3, i//3, i%3] = 0.001
		for _ in range(500):
			S = V2(I)
			A += S
		
		# Save aCalibration to file
		aCalibration = open('../Data/aCalibration.txt','w')
		for f in b:
			aCalibration.write(f'{f}\n')
		for r in A:
			for f in r:
				aCalibration.write(f'{f}\n')
		aCalibration.close()
		
		self.ab = b
		self.aA = A
	
	def mCalibrateCreate(self):
		D = np.array(self.mData[-500:])
		b = np.mean(D, axis=0)
		s = np.array([1,1,1])
		A = np.identity(3, dtype=float)
		S = np.identity(3, dtype=float)
		
		def v(p):
			return np.std(np.linalg.norm(b-D, axis=1)) - np.std(np.linalg.norm(p-D, axis=1))
		
		def V(M):
			return np.mean((np.linalg.norm(A @ D.T, axis=0) - R)**2) - np.mean((np.linalg.norm((A+M) @ D.T, axis=0) - R)**2)
		V2 = np.vectorize(V, signature='(m,n)->()')
		
		for _ in range(500):
			s = np.array(np.apply_along_axis(v, axis=1, arr = b+np.identity(3)*0.001)) * 1000
			b += s
		D -= b
		R = np.linalg.norm(D, axis=1)
		
		I = np.zeros((3,3,3,3), dtype=float)
		for i in range(9):
			I[i//3, i%3, i//3, i%3] = 0.001
		for _ in range(500):
			S = V2(I)
			A += S
		
		# Save mCalibration to file
		mCalibration = open('../Data/mCalibration.txt','w')
		for f in b:
			mCalibration.write(f'{f}\n')
		for r in A:
			for f in r:
				mCalibration.write(f'{f}\n')
		mCalibration.close()
		
		self.mb = b
		self.mA = A
	
	def gCalibrateCreate(self):
		D = np.array(self.gData[-100:])
		b = np.mean(D, axis=0)
		
		# Save gCalibration to file
		gCalibration = open('../Data/gCalibration.txt','w')
		for f in b:
			gCalibration.write(f'{f}\n')
		gCalibration.close()
		
		self.gb = b

def Orientation(self, a, m, g):

		self.q = self.madgwick.updateMARG(q=self.q, acc=a, mag=m, gyr=g)
		rotMatrix = sciR.from_quat(np.array([self.q[1], self.q[2], self.q[3], self.q[0]])).as_matrix()
		self.yaw = math.degrees(math.atan2(-rotMatrix[1, 0], rotMatrix[1, 1]))
		self.R = rotMatrix
		
		
		'''
		#print(np.dot(a, m))
		
		Z = self.Z
		R = self.R
		'''
		'''
		gGlobal = R.T @ g
		
		Zw = math.acos((np.trace(R)-1)/2)
		if (Zw == 0.0): Zw += 0.0001
		Zp = np.array([R[2][1] - R[1][2], R[0][2] - R[2][0], R[1][0] - R[0][1]])/(2*math.sin(Zw))
		Z = -Zp*Zw
		if (np.linalg.norm(Z) == 0.0): Z = np.array([0.0,0.0,1.0])
		'''
		'''
		x_ = np.array([1, 0, 0])
		y_ = np.array([0, 1, 0])
		z_ = np.array([0, 0, 1])
		
		A = self.u_
		M = self.n_
		
		def RodRot(v, p, w):
			return v + math.sin(w)*np.cross(p, v) + (1-math.cos(w))*np.cross(p, np.cross(p, v))
		
		def V(z):
			zm = np.linalg.norm(z)
			av2 = np.linalg.norm(RodRot(a, z/zm, zm) - A)**2
			mv2 = np.linalg.norm(RodRot(m, z/zm, zm) - M)**2
			return av2 + mv2
		
		def mat(p, w):
			return np.array([RodRot(x_, p, w), RodRot(y_, p, w), RodRot(z_, p, w)])
		'''
		'''
		def matg(M, p, w):
			return np.array([RodRot(M[0], p, w*dt), RodRot(M[1], p, w*dt), RodRot(M[2], p, w*dt)])
		
		rotMatrixG = matg(R, gGlobal/np.linalg.norm(gGlobal), np.linalg.norm(gGlobal))
		'''
		'''
		I = np.identity(3)/10000
		
		for i in range(10):
			Zstep = V(Z)*np.ones(3) - np.array([V(Z+I[0]), V(Z+I[1]), V(Z+I[2])])
			Z = Z + 900 * Zstep
		
		Zm = np.linalg.norm(Z)
		cycleReturn = Zm**(np.floor(Zm/(2*math.pi)))
		Z /= cycleReturn
		Zm = np.linalg.norm(Z)
		
		rotMatrixAM = mat(Z/Zm, Zm)
		'''
		'''
		# Gyro complimentary filter
		if (cycleReturn == 1):
			alpha = 0.25
			R = alpha*rotMatrixAM + (1.0-alpha)*rotMatrixG
		else:
			R = rotMatrixAM
		'''
		'''
		# Standard complimentary filter
		if (abs(cycleReturn-1.0) < 0.0001):
			alpha = 0.5
			R = alpha*rotMatrixAM + (1.0-alpha)*R
		else:
			R = rotMatrixAM
		'''
		'''
		# No filter
		R = rotMatrixAM
		
		self.R = R
		self.Z = Z
		'''
