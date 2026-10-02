
# Simple Inertial Measurement Unit (IMU) Test
# 
# Last edited :: Thursday, 27 March 2025
# By :: Hayden Graham
# 
# Electrical components used:
# - (1x) Raspberry Pi 4b
# - (1x) LSM6DSOX IMU
# 
# Wire connections from IMU to Raspberry:
# - Yellow/Yellow :: Ground aka Pin 9
# - Red/White     :: 5V aka Pin 2
# - Black         :: GPIO2 (SDA1) aka Pin 3
# - Blue/Blue     :: GPIO3 (SCL1) aka Pin 5

import board
import time
import math
import os
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX #the product name, 6d for 6 dimension

i2c = board.I2C() 
sox = LSM6DSOX(i2c)

timeStep = 0.05
alpha = 0.85
previousRoll = 0.0
previousPitch = 0.0

while True:
	
	accel = sox.acceleration #acceleration vector
	gyro = sox.gyro
	
	ax = accel[0] 
	ay = accel[1]
	az = accel[2]
	gx = gyro[0] #rotation
	gy = gyro[1]
	gz = gyro[2]
	mag = math.sqrt(ax**2 + ay**2 + az**2)
	
	os.system('clear')
	print("RawAccelX: " + str(round(ax, 2)))
	print("RawAccelY: " + str(round(ay, 2)))
	print("RawAccelZ: " + str(round(az, 2)))
	print("RawMagnitude: " + str(round(mag, 2)))
	print("RawRotVelX: " + str(round(gx, 2)))
	print("RawRotVelY: " + str(round(gy, 2)))
	print("RawRotVelZ: " + str(round(gz, 2)))
	
	predictedRoll = previousRoll + gx*timeStep
	measuredRoll = math.atan2(ay,az)
	estimatedRoll = alpha*predictedRoll + (1.0-alpha)*measuredRoll
	previousRoll = estimatedRoll
	
	predictedPitch = previousPitch + gy*timeStep
	measuredPitch = math.atan2(ax,math.sqrt(ay**2 + az**2))
	estimatedPitch = alpha*predictedPitch + (1.0-alpha)*measuredPitch
	previousPitch = estimatedPitch
	
	print("Measured Roll:  " + str(round(math.degrees(measuredRoll), 2)))
	print("Estimated Roll:  " + str(round(math.degrees(estimatedRoll), 2)))
	print("Measured Pitch:  " + str(round(math.degrees(measuredPitch), 2)))
	print("Estimated Pitch: " + str(round(math.degrees(estimatedPitch), 2)))
	'''
	print("Normal 'X': " + str(x)) #Relative to the breadboard, X changes when tilted forwards and backwards(the short ends)
	print("Normal 'Y': " + str(y)) #Relative to the breadboard, Y changes when tilted left and right(the long sides)
	print("Normal 'Z': " + str(z)) #Relative to the breadboard, Z changes when moving up and down (when not moving, Z is +10)
	'''
	
	time.sleep(timeStep)
