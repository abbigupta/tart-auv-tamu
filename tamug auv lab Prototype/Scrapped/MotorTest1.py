
# Simple 1st Motor Test
# 
# Last edited :: Thursday, 20 March 2025
# By :: Hayden Graham
# 
# Electrical components used:
# - (1x) Raspberry Pi 4b
# - (1x) 30A Bidirectional ESC
# - (1x) Blue Robotics Waterproof Motor
# - (1x) Power supply
# 
# Wire connections from ESC to Raspberry:
# - Brown  :: Ground aka Pin 39
# - Red    :: nothing (DO NOT CONNECT)
# - Yellow :: GPIO19 aka Pin 35
# 
# Power supply was set to 12 Volts, and ESC was hooked up to the leads
# ESC and Motor were soldered together
# 
# Details about ESC: ((REQUIRES FURTHER TESTING TO BE SURE))
# - Operating Frequency :: 6000 Hertz
# - Duty Cycle = ???    :: Full reverse
# - Duty Cycle = ???    :: Zero throttle
# - Duty Cycle = ???    :: Full forward
#
# With current ESC, thrust appears to plateau at around Duty Cycle [DC] value of 30.
# Methods used were not very scientific or numerical, so take this estimation with an ocean of salt
# Generated thrust was at least not insignificant, should be sufficient to maneuver the AUV
# More precise measurements to be performed once testing rig is built

#import board
import time
import math
import RPi.GPIO as io
import os

print(io.getmode())

# GPIO pin setup
io.setmode(io.BOARD)
io.setwarnings(False)

# Leading pin we're using
leadpin = 35 #GPIO19
leanpin2 = 36

# Set the lead pin (35) as output
io.setup(leadpin, io.OUT)
io.setup(leadpin2, io.OUT)

# PWM signal details
frequency = 6000 #hz; it does not really matter
dutyCycle = 0.0  #50 is half up/down
dutyCycle2 = 0.0  #50 is half up/down

# Setup the PWM signal generation object
Motor1 = io.PWM(leadpin, frequency)
Motor2 = io.PWM(leadpin2, frequency)

# Start Program
print('Waiting 2s for startup...')
time.sleep(2)

# Start motor at 0% dutyCycle
Motor1.start(0)
Motor2.start(0)

#time.sleep(10)

for i in range(1):
	for dc in range(0,51,2):
		
		dutyCycle = dc
		print('Duty Cycle :: ' + str(dutyCycle) + '%')
		Motor1.ChangeDutyCycle(dutyCycle)
		
		dutyCycle2 = 50-dc
		print('Duty Cycle :: ' + str(dutyCycle2) + '%')
		Motor2.ChangeDutyCycle(dutyCycle2)
		
		time.sleep(2)

#for i in range(1):
#	for dc in range(0,101,5):
#		dutyCycle = dc
#		print('Duty Cycle :: ' + str(100.0 - dutyCycle) + '%')
#		Motor1.ChangeDutyCycle(100.0 - dutyCycle)
#		time.sleep(0.5)


#for i in range(1):
#	for dc in range(0,11,1):
#		dutyCycle = dc
#		print('Duty Cycle :: ' + str(dutyCycle/1000.0) + '%')
#		Motor1.ChangeDutyCycle(dutyCycle/1000.0)
#		time.sleep(1)

# Stop Motor
Motor1.stop()
Motor2.stop()
time.sleep(2)

print('End of program')

