import board
import math
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX

class IMU:
	
	ax = 0.0
	ay = 0.0
	az = 0.0
	gx = 0.0
	gy = 0.0
	gz = 0.0
	
	magnitude = 0.0
	
	pitch = 0.0
	roll = 0.0
	yaw = 0.0
	
	acceptMagYaw = False
	filterWeight = 0.85
	timeStep = 0.05
	
	def __init__(self, sox = LSM6DSOX(board.I2C())):
		self.sox = sox
	
	def printRawData(self):
		print(f"RawAccelX: {self.ax:.2f}")
		print(f"RawAccelY: {self.ay:.2f}")
		print(f"RawAccelZ: {self.az:.2f}")
		print(f"RawRotVelX: {self.gx:.2f}")
		print(f"RawRotVelY: {self.gy:.2f}")
		print(f"RawRotVelZ: {self.gz:.2f}")
	
	def printData(self):
		print('\n---IMU---------------------')
		#print(f"Magnitude: {self.magnitude:.2f}")
		print(f"Roll: {math.degrees(self.roll):.2f}")
		print(f"Pitch: {math.degrees(self.pitch):.2f}")
		print(f"Yaw: {math.degrees(self.yaw):.2f}")
	
	def getPitch(self):
		return self.pitch
	
	def getRoll(self):
		return self.roll
	
	def getYaw(self):
		return self.yaw
	
	def acceptMag(self):
		self.acceptMagYaw = True
	
	def tick(self, magYaw):
		
		accel = self.sox.acceleration
		gyro = self.sox.gyro
		
		ax = accel[0]
		ay = accel[1]
		az = accel[2]
		gx = gyro[0]
		gy = gyro[1]
		gz = gyro[2]
		
		self.ax = accel[0]
		self.ay = accel[1]
		self.az = accel[2]
		self.gx = gyro[0]
		self.gy = gyro[1]
		self.gz = gyro[2]
		self.magnitude = math.sqrt(ax**2 + ay**2 + az**2)
		
		preRoll = self.roll + gx*self.timeStep
		newRoll = math.atan2(ay,az)
		bestRoll = self.filterWeight*preRoll + (1.0-self.filterWeight)*newRoll
		self.roll = bestRoll
		
		prePitch = self.pitch + gy*self.timeStep
		newPitch = math.atan2(ax,math.sqrt(ay**2 + az**2))
		bestPitch = self.filterWeight*prePitch + (1.0-self.filterWeight)*newPitch
		self.pitch = bestPitch
		
		preYaw = self.yaw + gz*self.timeStep
		newYaw = self.yaw
		if self.acceptMagYaw: newYaw = magYaw
		bestYaw = self.filterWeight*preYaw + (1.0-self.filterWeight)*newYaw
		self.yaw = bestYaw
		self.acceptMagYaw = False

