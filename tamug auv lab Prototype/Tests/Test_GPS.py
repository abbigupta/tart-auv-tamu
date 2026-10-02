#!/home/tamugauv/venv01/bin/python
# Simple Global Positioning System (GPS) Test
# 
# Last edited :: Thursday, 3 April 2025
# By :: Hayden Graham
# 
# Electrical components used:
# - (1x) Raspberry Pi 4b
# - (1x) GT-U7 GPS Module
# 
# Wire connections from GPS to Raspberry:
# - Yellow   :: 5V aka Pin 4
# - Gray     :: Ground aka Pin 6
# - Yellow   :: GPIO14 (TXD) aka Pin 8
# - Purple   :: GPIO15 (RXD) aka Pin 10

import serial
import pynmea2
from datetime import datetime
import time

# Whether or not to save logs to file
save = False

# Configure serial port
port = "/dev/ttyS0" # Or /dev/ttyAMA0 for some models; use tty1 for permission; serial0
baudrate = 9600
ser = serial.Serial(port, baudrate, timeout=0.5)

if save: file_Logs = open('../Data/GPS_logs.txt','w')
if save: file_NMEA = open('../Data/GPS_NMEA.txt','w')

for i in range(20000):
	
	time.sleep(0.01)
	timestamp = str(datetime.now())
	
	try:
		
		# Pull NMEA statement from GPS
		line = ser.readline().decode("utf-8")
		print(line)
		
		# Check if it's a valid NMEA-RMC sentence
		if line.startswith('$GPRMC'):
			
			try:
				
				# Parse the NMEA sentence
				msg = pynmea2.parse(line)
				lat = msg.latitude
				lon = msg.longitude
				
				# Save data to files
				if save: file_NMEA.write(line)
				if save: file_Logs.write(f"{timestamp} || Lat: {lat:.6f}, Lon: {lon:.6f}\n")
				
				# Print the data
				print(f"Lat: {lat:.6f}   Lon: {lon:.6f}")
				
			except pynmea2.ParseError as e:
				print(f"Error: {e}")
	
	except Exception as e:
		print(end='')
		#print(f"Parse error: {e}")

if save: file_Logs.close()
if save: file_NMEA.close()

if save: print('Saved Coordinate logs in: \"','Data/GPS_logs.txt','\"',sep='')
if save: print('Saved NMEA logs in: \"','Data/GPS_NMEA.txt','\"',sep='')
