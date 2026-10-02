
# Simple 1st Motor Test
# 
# Last edited :: Wednesday, 23 April 2025
# By :: Hayden Graham
# 
# Electrical components used:
# - (1x) Raspberry Pi 4b
# - (1x) 45A ApisQueen Bidirectional ESC
# - (1x) Blue Robotics Waterproof Motor
# - (1x) Power supply
# 
# Wire connections from ESC to Raspberry:
# - Black :: Ground aka Pin 39
# - White :: GPIO19 aka Pin 35
# 
# Power supply was set to 12 Volts, and ESC was hooked up to the leads
# ESC and Motor were soldered together
# 
# Details about ESC: ((REQUIRES FURTHER TESTING TO BE SURE))
# - Operating Frequency :: Up to 500 Hertz
# - Duty Cycle = 50    :: Full reverse
# - Duty Cycle = 75    :: Zero throttle
# - Duty Cycle = 100    :: Full forward
#
# New ESC has clear directions on how it works, hard part is powering it.
# Once power isn't an issue, tests should be able to commence immediately
# - Haowei Oh

#import board
import time
#import math
import RPi.GPIO as io
#import os

# GPIO pin setup
io.setmode(io.BOARD)
io.setwarnings(False)

# Leading pin we're using
leadpin = 35 #GPIO19

# Set the lead pin (35) as output
io.setup(leadpin, io.OUT)

# PWM signal details
frequency = 500
dutyCycle = 72

# Setup the PWM signal generation object
Motor1 = io.PWM(leadpin, frequency)

# Start Program
#print('Waiting 1s for startup...')
#time.sleep(0.1)

# Start motor at 73% dutyCycle to arm
Motor1.start(dutyCycle)
print(f'ms :: {dutyCycle/50}')

#print('Waiting 3s for startup2...')
time.sleep(1)

for i in range(1):
	for dc in range(70,76,1):
		dutyCycle = dc
		print(f'ms :: {dutyCycle/50}')
		Motor1.ChangeDutyCycle(dutyCycle)
		time.sleep(7)

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
time.sleep(2)

# Stop Motor
Motor1.stop()

print('End of program')

