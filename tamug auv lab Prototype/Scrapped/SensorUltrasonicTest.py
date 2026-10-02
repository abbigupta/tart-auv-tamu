import board
import time
import math
import RPi.GPIO as io
import os

io.setmode(io.BCM)

trigger = 23
echo = 24

# Setup the input pin and the output pin
io.setup(trigger, io.OUT)
io.setup(echo, io.IN)

# Let sensor settle before taking input
io.output(trigger, False)
time.sleep(2)
dataDictionary = {}
for i in range(1000):
	dataDictionary[str(i)] = []
print(dataDictionary)
#while(len(dataList) < 1000):
for i in range(1000):
	try:
		# Send 10 microsecond pulse to start the sensor
		io.output(trigger, True)
		time.sleep(0.00001)
		io.output(trigger, False)
		
		
		while (io.input(echo) == 0):
			continue

		pulse_start = time.time()

		while (io.input(echo) == 1):
			continue

		pulse_end = time.time()

		pulse_duration = pulse_end - pulse_start

		# 17150 is in centimeters per second
		distance_to_object = round(17150 * pulse_duration, 6)
		#if(distance_to_object > 50):
		#	distance_to_object = 50
		index = int(distance_to_object//0.05)-1
		if(index < 200):
			index = 0
		if(index >= 301):
			index = 301
		dataDictionary[str(index)].append(distance_to_object)
		os.system('clear')
		#if (0 < distance_to_object < 55):
		print('distance found was',distance_to_object,'centimeters.')
		#else:
		#	print('sensor error')
	except:
		print('Sensor ping failure')
	time.sleep(0.015)
print(dataDictionary)
for i in range(200,301):
	print(i,' :: ',len(dataDictionary[str(i)]))
#for i in range(20):
	#print(len(dataList[i]))
#for i in range(100):
	#print(dataList[0][i] == dataList[1][i])
#print(round(sum(dataList)/len(dataList), 2))
