
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

# Wire connections from ESC to Raspberry: (the mortor)
# - Brown  :: Ground aka Pin 39
# - Red    :: nothing (DO NOT CONNECT)
# - Yellow :: GPIO19 aka Pin 35


import time
import math
import os
import RPi.GPIO as io
print(io.getmode())
import board
from scipy.spatial.transform import Rotation as R
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX #the product name, 6d for 6 dimension

i2c = board.I2C() 
sox = LSM6DSOX(i2c)

timeStep = 0.05  #
alpha = 0.85
previousRoll = 0.0
previousPitch = 0.0

#===== setup for the motor
# GPIO pin setup
#print(dir(io))
#print(io.getmode(),io.BOARD)
#io.setmode(io.BOARD)
io.setwarnings(False)

# Leading pin we're using
leadpin = 19 #GPIO19 in mode 11; when in mode 10, it is 35; 

# Set the lead pin (35) as output
io.setup(leadpin, io.OUT)

# PWM signal details
frequency = 6000 #hz; it does not really matter
dutyCycle = 0.0  #50 is half up/down

# Setup the PWM signal generation object
Motor1 = io.PWM(leadpin, frequency)

# Start Program
print('Waiting 2s for startup...')
time.sleep(2)

# Start motor at 0% dutyCycle
Motor1.start(0)

#=== end of setup for motor 

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
	
	if math.degrees(measuredPitch)> 30: 
		Motor1.ChangeDutyCycle(20)
	else:
		Motor1.ChangeDutyCycle(0)
	#time.sleep(2)
	
	
	time.sleep(timeStep)
