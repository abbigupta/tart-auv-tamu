import serial
import pynmea2
import numpy as np
import math
from math import sin,cos,atan2
from scipy.stats import linregress
from functions import diff_yaw,mean_yaw,std_yaw,calculate_yaw
from collections import deque
from datetime import datetime
import os

def get_direction(u,v):
	return np.rad2deg(atan2(u,v))

class GPS:
	
	port = '/dev/serial0'
	baudrate = 9600
	ser = serial.Serial(port, baudrate, timeout=0.5)
	
	NMEA = ''
	latitude = -1.0
	longitude = -1.0
	hasFix = False
	lons = deque([], maxlen=100) #saving 100 recent readings with first item the newest
	lats = deque([], maxlen=100)
	yaws = deque([-999], maxlen=100) #yaw for each tick in the past
	distances = deque([0], maxlen=100) #yaw for each tick in the past
	yaw_avg = -999 #yaw avg over past few seconds
	itick = 0 
	tps = 5 #tick per second
	ntick_avg = 5 # ntick_avg records will be avg for lon/lat; if 0, instaneous reading will be provided; use integer
	MIN_MOVEMENT = 0.5  # minimum movement in meters to consider valid heading

	# for writing
	write_data = False
	writing_dir = '../Data/GPSlogs/'
	fid = None
	writing_interval = 1 # in seconds
	previous_writing_time = datetime.now()
	
	def getYaw(self,option=1,ntime=5,max_std=2): #yaw for the last reading
		if option==1:
			# Need at least 2 seconds of data
			if len(self.lons) < 2 * self.tps:
				return -999
				
			# Calculate average positions for current and previous second
			lon_now = np.mean(np.array(self.lons)[0:self.tps])
			lat_now = np.mean(np.array(self.lats)[0:self.tps])
			lon_previous = np.mean(np.array(self.lons)[self.tps:2*self.tps])
			lat_previous = np.mean(np.array(self.lats)[self.tps:2*self.tps])
				
			# Calculate distance moved
			dx = (lon_now - lon_previous) * cos(math.radians(lat_now)) * 111320  # Convert to meters
			dy = (lat_now - lat_previous) * 111320  # Convert to meters
			distance = math.sqrt(dx*dx + dy*dy)
			self.distances.appendleft(distance)
				
			# If movement is too small, consider it stationary
			if distance < self.MIN_MOVEMENT:
				return -999
				
			# Calculate yaw from GPS positions
			yaw_now = calculate_yaw(lon_previous, lat_previous, lon_now, lat_now)

		if option == 2: #get multiple seconds of gps data, only get yaw when the line is straight
			if len(self.lons) < ntime * self.tps:
				return -999
				
			lons = np.array(self.lons)[0:ntime*self.tps]
			lats = np.array(self.lats)[0:ntime*self.tps]
			lons = lons.reshape([ntime,self.tps]).mean(axis=1) #average every second
			lats = lats.reshape([ntime,self.tps]).mean(axis=1) #average every second
			# get mean and std of gps and current yaws 
			gyaws = [] #get GPS yaws; do not use the yaw from first to final; as in some case, the vessle may reverse back (still straight line)
			for ii in range(len(lons)-1):
				gyaws.append(calculate_yaw(lons[ii+1], lats[ii+1], lons[ii],lats[ii])) #get direction from (lon1,lat1) to (lon2,lat2)
			std_gyaw=std_yaw(gyaws)
			if std_gyaw<max_std:
				yaw_now = mean_yaw(gyaws)
			else:
				yaw_now = -999
            
		# Add to yaw history
		self.yaws.appendleft(yaw_now)
            
		return yaw_now
		
	def getYawAvg(self,n=10,stable_std=10): #get yaw based on GPS reading
		if len(self.yaws)<n: self.yaw_avg = -999; return
		x=np.array(self.yaws)[:n] 
		stdx = std_yaw(x)
		if stdx > stable_std: self.yaw_avg = -999; return
		self.yaw_avg = mean_yaw(x)
	
	def printRawData(self):
		print(f'NMEA: {self.NMEA}')
	
	def printData(self):
		#print('\n---GPS---------------------')
		print(f'[GPS] Lat: {self.latitude:.6f}; Lon: {self.longitude:.6f}; Yaw: {self.yaws[0]:.2f}; movement: {self.distances[0]:.2f}')
		#print(f'# of yaws: {len(self.yaws)-1}')
		#if len(self.yaws)>10: print(f'std of last 10 yaws: {std_yaw(np.array(self.yaws)[0:10]):.0f}')
		#print(f'Longitude: {self.longitude:.6f}')
		#print(f'Yaw: {self.yaw:.2f}')
		#print('Valid GPS readings',sum(self.lons!=0))
	
	def writeData(self):
		if self.write_data: #enable writing, e.g., in the water
			if self.fid==None or self.fid.closed:
				if os.path.exists(self.writing_dir): os.makedirs(self.writing_dir,exist_ok=True)
				timestamp = datetime.now().strftime('%Y%m%d%H%M')
				fname=f'{self.writing_dir}/GPSlogs_{timestamp}.csv'
				print(f'Opening {fname}')
				self.fid = open(fname,'w')
				self.fid.write('time,lat,lon\n')
			elif (datetime.now() - self.previous_writing_time).seconds >= self.writing_interval:
				timestamp = datetime.now().strftime('%Y%m%d%H%M%S')
				line = f'{timestamp},{self.longitude:.8f},{self.current_latitude:.8f}\n'
				self.fid.write(line)
				self.previous_writing_time = datetime.now()
		else:
			if self.fid != None and not self.fid.closed: self.fid.close()

	def getCoordinates(self):
		return self.coordinates
	
	def tick(self):
		self.itick += 1
		try:
			NMEA = self.ser.readline().decode("utf-8")
			
			if NMEA.startswith('$GPRMC'):
				try:
					msg = pynmea2.parse(NMEA)
					self.NMEA = NMEA
					self.latitude = float(msg.latitude) #instaneous
					self.longitude = float(msg.longitude) #instaneous
					if (not (self.latitude in [0.0, -1.0])):
						self.hasFix = True
						# add current reading to the beginning of lons and lats
						self.lons.appendleft(self.longitude)
						self.lats.appendleft(self.latitude)

				except pynmea2.ParseError as e:
					print(f"Error: {e}")
			if self.ntick_avg>1 and len(self.lons)>=self.ntick_avg: 
				self.longitude = np.mean(np.array(self.lons)[0:self.ntick_avg])
				self.latitude = np.mean(np.array(self.lats)[0:self.ntick_avg])
			
			self.getYaw() #gps yaw from locaiton at previous second to location now

			self.writeData() #only write when self.write_data is True, controlled in the Simutaneous.py
		
		except Exception as e:
			print(f"Parse error: {e}")
