import numpy as np
import math
from geopy.distance import distance as g_distance
from geopy.distance import geodesic as g_geodesic
from geopy import Point as g_Point
import time
from collections import deque
from functions import diff_yaw,mean_yaw,std_yaw,calculate_yaw
from scipy.stats import linregress
from datetime import datetime
import os
import pandas as pd

class PID:
	def get_yll(self): #get yaw, lon, lat, distance
		# yaw and lon, lat updated in sim.py
		self.target_yaw = self.compute_target_yaw()
		self.error_yaw = self.compute_error_yaw()
		self.error_distance = self.current_distance
		# need lons,lats,yaws,and distances for GPS yaw correction
		self.distances.appendleft(self.error_distance)
		self.current_lons.appendleft(self.current_lon)
		self.current_lats.appendleft(self.current_lat)
		self.current_yaws.appendleft(self.current_yaw)

	def __init__(self, movement_distance=10, movement_direction=-90, drive_gains=[0.2, 0.0, 0.0], turn_gains=[0.01, 0.0, 0.01], dt=0.05,
	            fname_coords='../Data/target_coords.csv',success_distance=3, yaw_correction_with_gps=0, yaw_source='amg',delta_turn = 0.1,
	            delta_drive=0.01):
		
		# Time step for PID (seconds)
		self.dt = dt
		
		# Distance of planned movement (meters)   #not used anymore
		self.movement_distance = movement_distance
		
		# Direction of planned movement (degrees CW from north) #not used anymore
		self.movement_direction = movement_direction
		
		# First viable GPS coords
		self.start_lat = 0.0
		self.start_lon = 0.0
		
		# Current GPS coords
		self.current_lat = 0.0
		self.current_lon = 0.0
		
		# Target GPS coords lists		
		tmp=np.array([line.strip().split(',')  for line in open(fname_coords, 'r').readlines()]).astype('float')
		self.target_lat_list = list(tmp[:,0]) #must be a list
		self.target_lon_list = list(tmp[:,1]) #must be a list. pop(0) function will be called 
		
		
		# Target GPS coords
		self.target_lat = self.target_lat_list.pop(0)
		self.target_lon = self.target_lon_list.pop(0)
		
		# Current Distance and Yaw
		self.current_distance = 0.0
		self.current_yaw = 0.0
		
		# Target Distance and Yaw
		self.target_distance = 0.0
		self.target_yaw = 0.0
		
		# Errors in Distance and Yaw
		self.error_distance = 0.0
		self.error_yaw = 0.0
		
		# Previous Errors in Distance and Yaw (Used for Derivative estimate)
		self.previous_error_distance = 0.0
		self.previous_error_yaw = -999
		
		# PID control
		self.has_start_coords = False
		self.drive = 0.0
		self.turn = 0.0
		self.success = False #False until get the final stop
		self.success_distance=success_distance
		
		# PID control second method; tick2()
		self.yaw_source = yaw_source
		self.turn_class = 10
		self.fix_turn = 0.1 #fix_turn; must be positive; it will be multiply with np.sign(error_yaw)
		self.dps =     {10:6, 20:10,  30:15}       #degree turned per second with for different turn class 10, 20, 30 
		self.dps_min = {10:6,  20:10,  30:15}    #minimum dps for different turn class
		self.dps_max = {10:6, 20:10,  30:15}   #maximum dps for different turn class
		self.dps2 = 60      #degree turned per second with turn of 1 and drive of 0, used when it hit a stop; try left turn key and see how much it will need to turn 180; 180/the time.  
		self.ttime = 0      #turn time with turn of 0.1
		self.ttimes = deque([],maxlen=500)
		self.pid2_interval = 1 #number of seconds for full drive after the turn. 


		# PID gains for drive output
		self.Kp_drive = drive_gains[0]
		self.Ki_drive = drive_gains[1]
		self.Kd_drive = drive_gains[2]
		
		# PID gains for turn output
		self.Kp_turn = turn_gains[0]
		self.Ki_turn = turn_gains[1]
		self.Kd_turn = turn_gains[2]
		self.delta_turn = delta_turn  #to limit dramatic change of turn
		self.delta_drive = delta_drive  #to limit dramatic change of turn
		
		# Minimum and Maximum output
		self.MIN_OUTPUT = -1.0
		self.MAX_OUTPUT = 1.0

		self.itick = 0
		self.itick2 = 0
		self.turns = deque([],maxlen=500)
		self.drives = deque([],maxlen=500)
		self.drives.appendleft(0)
		self.turns.appendleft(0)
		self.current_yaws = deque([],maxlen=500)
		self.current_lons = deque([],maxlen=500)
		self.current_lats = deque([],maxlen=500)
		self.distances = deque([],maxlen=500)
		self.stuck = 0
		self.stuck_lon = 0
		self.stuck_lat = 0
		self.stuck_yaw = 0 
		self.stuck_itick = 0
		self.stuck_distance = 0 
		self.stuck_direction = 1 #what direction to move when stuck
		self.stuck_nstop = len(self.target_lat_list)
		self.enabled = False
		self.yaw_offset_from_gps = 0
		self.itick_gps_correction = 0
		self.yaw_correction_with_gps = yaw_correction_with_gps
		self.gps_yaw_rvalue = 0
		self.mean_cyaw=0.0
		self.mean_gyaw=0.0
		self.std_cyaw=0.0
		self.std_gyaw=0.0
		self.distance5=0.0

		# for writing 
		self.status = 0
		self.fid = None
		self.writing_interval = 1 # in seconds
		self.previous_writing_time = datetime.now()
		self.writing_dir = '../Data/PIDlogs/'
		if os.path.exists(self.writing_dir): os.makedirs(self.writing_dir,exist_ok=True)
		self.data = {'time':[],'stops':[],'lat':[],'lon':[],'distance':[],'drive':[],'target_yaw':[],'current_yaw':[],
			   'turn':[],'gps_offset':[],'rvalue':[],'gyaw':[],'cyaw':[],'std_gyaw':[],'std_cyaw':[],'distance5':[],
			   'stuck':[],'ttime':[],'dps':[],'turn_class':[]}
	
	def printData(self):
		
		print(f'[PID] enabled:{self.enabled}; {len(self.target_lat_list)+1} STOPS remaining')
		#print(f'current_lat: {self.current_lat:.6f}; current_lon: {self.current_lon:.6f}')
		print(f'[PID] target_lat: {self.target_lat:.6f}; target_lon: {self.target_lon:.6f}')
		print(f'[PID] distance: {self.error_distance:.1f} (m) | drive: {self.drive:.1f}')
		print(f'[PID] current_yaw: {self.current_yaw}   | target_yaw: {self.target_yaw:.0f}')
		print(f'[PID] error_yaw: {self.error_yaw}     | turn: {self.turn:.2f}')
		print(f'[PID] turn class: {self.turn_class}; tps: {self.dps[self.turn_class]:.1f} | turn time: {self.ttime:.2f}')
		print(f'[PID] gps_yaw_rvalue: {self.gps_yaw_rvalue:.2f}; yaw offset from gps:{self.yaw_offset_from_gps:.0f}')
		print(f'[PID] mean_AMG_yaw: {self.mean_cyaw:.2f}, mean_GPS_yaw: {self.mean_gyaw:.2f}')
		print(f'[PID] std_AMG_yaw: {self.std_cyaw:.2f}, std_GPS_yaw: {self.std_gyaw:.2f}')
		print(f'[PID] distance moved in 5 seconds: {self.distance5:.1f}')
		print(f'')
		
		#print()
		#print(f'has_start_coords: {self.has_start_coords}')
		
		#print('success',self.success)
		#print('stuck',self.stuck)
		#print('distance #',len(self.distances))

	def write_data(self,fmt=0):
		if self.enabled:
			self.status = 1 
			if fmt==0: #writing into csv
				if self.fid==None or self.fid.closed:
					timestamp = datetime.now().strftime('%Y%m%d%H%M')
					fname=f'{self.writing_dir}/PIDlogs_{timestamp}.csv'
					print(f'Opening {fname}')
					self.fid = open(fname,'w')
					self.fid.write('time,stops,lat,lon,distance,drive,target_yaw,current_yaw,turn,gps_offset,rvalue,gyaw,cyaw,std_gyaw,std_cyaw,distance5,stuck,turn_class,ttime,dps\n')
				# write information to csv file
				if (datetime.now() - self.previous_writing_time).seconds >= self.writing_interval:
					timestamp = datetime.now().strftime('%Y%m%d%H%M%S')
					line = f'{timestamp},{len(self.target_lon_list)},{self.current_lat:.6f},{self.current_lon:.6f},{self.current_distance:.2f},{self.drive:.2f},'+\
						f'{self.target_yaw:.2f},{self.current_yaw:.2f},{self.turn:.2f},'+\
						f'{self.yaw_offset_from_gps:.2f},{self.gps_yaw_rvalue:.2f},{self.mean_gyaw:.2f},{self.mean_cyaw:.2f},{self.std_gyaw:.2f},{self.std_cyaw:.2f},{self.distance5:.2f},'+\
						f'{self.stuck},{self.turn_class},{self.ttime:.2f},{self.dps[self.turn_class]:.2f}\n'
					self.fid.write(line)
					self.previous_writing_time = datetime.now()
			elif fmt==1: #collect data and write the data at the end 
				now = datetime.now()
				timestamp = now.strftime('%Y%m%d%H%M%S')
				if (now - self.previous_writing_time).seconds >= self.writing_interval: 
					self.data['time'].append(timestamp)
					self.data['stops'].append(len(self.target_lon_list))
					self.data['lat'].append(self.current_lat)
					self.data['lon'].append(self.current_lon)
					self.data['distance'].append(self.current_distance)
					self.data['drive'].append(self.drive)
					self.data['turn'].append(self.turn)
					self.data['target_yaw'].append(self.target_yaw)
					self.data['current_yaw'].append(self.current_yaw)
					self.data['gps_offset'].append(self.yaw_offset_from_gps)
					self.data['rvalue'].append(self.gps_yaw_rvalue)
					self.data['gyaw'].append(self.mean_gyaw)
					self.data['cyaw'].append(self.mean_cyaw)
					self.data['std_cyaw'].append(self.std_cyaw)
					self.data['std_gyaw'].append(self.std_gyaw)
					self.data['distance5'].append(self.distance5)
					self.data['stuck'].append(self.stuck)
					self.data['turn_class'].append(self.turn_class)
					self.data['ttime'].append(self.ttime)
					self.data['dps'].append(self.dps[self.turn_class])
					self.previous_writing_time = now
		elif self.status == 1: #!!Todo: need to deal with when the program is keyboard abrupted
			if self.fid != None and not self.fid.closed: self.fid.close()
			if fmt==1:  #!! need a condition when previous enabled is on
				timestamp = datetime.now().strftime('%Y%m%d%H%M%S')
				df = pd.DataFrame(self.data)     # or pd.DataFrame(list_of_dicts)
				df.to_csv(f'{self.writing_dir}/PIDlogs_{timestamp}.csv', index=False)
			self.status = 0 

	# Find coordinates of target location
	def compute_target_coords(self):
		target = g_distance(meters=self.movement_distance).destination(point=g_Point(self.start_lat, self.start_lon), bearing=self.movement_direction)
		return target.latitude, target.longitude
	
	# Find desired yaw to point towards target location
	def compute_target_yaw(self):
		dLon = math.radians(self.target_lon - self.current_lon)
		y = math.sin(dLon) * math.cos(math.radians(self.target_lat))
		x = math.cos(math.radians(self.current_lat)) * math.sin(math.radians(self.target_lat)) - \
			math.sin(math.radians(self.current_lat)) * math.cos(math.radians(self.target_lat)) * math.cos(dLon)
		return (math.degrees(math.atan2(y, x)) + 360) % 360
	
	# Finds closest error in yaw from target yaw
	def compute_error_yaw(self):
		if self.current_yaw == -999: return -999
		return diff_yaw(self.target_yaw,self.current_yaw)
	
	def clampOutput(self, output):
		return max(min(output, self.MAX_OUTPUT), self.MIN_OUTPUT)
	
	# Find distance to target location
	def compute_distance_to_target(self):
		return g_geodesic((self.current_lat, self.current_lon), (self.target_lat, self.target_lon)).meters
	
	# Take coordinates from gps loop
	def takeCoords(self, lat, lon):
		if (not self.has_start_coords):
			self.start_lat, self.start_lon = lat, lon
			self.has_start_coords = True
		self.current_lat, self.current_lon = lat, lon
		self.current_distance = self.compute_distance_to_target()
	
	# Take yaw from amg loop
	def takeYaw(self, yaw):
		if yaw == None or yaw==-999: 
			self.current_yaw = -999
		else:
			self.current_yaw = (yaw+self.yaw_offset_from_gps)%360 # yaw here from AMG, need add gps correction

	# Tick the PID with latest response variables
	def tick(self):
		self.itick += 1
		self.itick2 += 1 #tick after reaching a stop point
		
		# Get target yaw
		self.target_yaw = self.compute_target_yaw()
		
		# Errors
		self.error_distance = self.current_distance
		self.distances.appendleft(self.error_distance)
		self.current_lons.appendleft(self.current_lon)
		self.current_lats.appendleft(self.current_lat)
		self.current_yaws.appendleft(self.current_yaw)

		self.get_yaw_offset_with_gps()  #regardless of turn it on or not; it will calculate gps yaw and current yaws, useful for testing
		
		self.error_yaw = self.compute_error_yaw()

		# Control (P only for now) on drive
		# give 5 seconds for the robot to have stronger turn while without drive after hitting a stop
		if self.itick2<100 and abs(self.error_yaw)>30:
			self.drive = 0
			self.turn = self.Kp_turn * self.error_yaw *2 
		else:
			self.turn = self.Kp_turn * self.error_yaw
			self.drive = self.Kp_drive * self.error_distance

		# add derivative turn, Kd_turn*(current_error - previous_error)/dt
		if self.Kd_turn != 0.0 and self.previous_error_yaw != -999: 
			self.turn += (self.error_yaw - self.previous_error_yaw)/self.dt*self.Kd_turn

		# trying to limit jumpy turns
		if len(self.turns)>0: 
			self.turn = np.clip(self.turn, self.turns[0]-self.delta_turn,self.turns[0]+self.delta_turn)
			
		if len(self.drives)>0: 
			self.drive = np.clip(self.drive, self.drives[0]-self.delta_drive,self.drives[0]+self.delta_drive)
			
		# make the turn and drive within range of (-1,1)
		self.drive = np.clip(self.drive,-1,1)
		self.turn = np.clip(self.turn,-1,1)
		
		if not self.enabled: self.drive = 0  #need this for smooth start
		
		# Limit turn when in large distance, avoid over shooting; only apply after itick>200 (meaning 10s after PID being enabled or starting heading to next destination)
		#max_turn = 1/min([10,self.error_distance-self.success_distance])  #absolute value; closer to destination, larger maximum turn
		#if abs(self.turn) >0.10 and self.itick<200 : self.turn = min([max_turn, abs(self.turn)])*self.turn/abs(self.turn)

		self.drives.appendleft(self.drive)
		self.turns.appendleft(self.turn)
		
		# Previous errors; used for D control (not yet implemented yet)
		self.previous_error_distance = self.error_distance
		self.previous_error_yaw = self.error_yaw
		
		# Success condition
		self.success = (self.error_distance < self.success_distance)
		
		# if get stuck, 10 seconds, 200 readings
		if False and self.enabled and self.itick-self.stuck_itick>200 and len(self.target_lat_list)>0: # and self.error_distance < self.stuck_distance:
			tmp = np.array(self.distances)[0:100] #use 100 tick instead of 200 tick
			changes = abs(tmp - tmp[-1])
			move_distance = 5 #distance to move when stuck 
			if sum(changes>1)==0: #no move at all, meaning stuck
				self.stuck = 1
				self.stuck_itick = self.itick
				# compared to previous stucking location; 
				# first move to right; if move by >1m, meaning right direction is not block; keep moving
				# if move by <1m from stucking, meaning block in right direction; future moving direction to left, until blocked again in left direction
				if abs(self.error_distance-move_distance)<1:  #when the error distance say 4m now, meaning the robot does not move over 1m over the past 5 seconds
					self.stuck_direction *= -1 #change the direction when being stuck
					target = g_distance(meters=move_distance).destination(point=g_Point(self.stuck_lat, self.stuck_lon), bearing=(self.stuck_yaw+90*self.stuck_direction)%360) #second time, move to left; third time, move to right; ..
					self.target_lat,self.target_lon = target.latitude, target.longitude
					self.target_lat_list[0]=self.target_lat #change the new stop; don't add new stop  
					self.target_lon_list[0]=self.target_lon 
					print('!!!\n!###No move, stuck, switch direction\n!!!')
				else:
					print('!!!\n!!!!stuck\n!!!')
					self.stuck_lon = self.current_lon
					self.stuck_lat = self.current_lat
					self.stuck_yaw = self.target_yaw
					self.stuck_distance = self.error_distance
					self.stuck_nstop = min([self.stuck_nstop,len(self.target_lat_list)]) #nstop will be smaller and smaller
					target = g_distance(meters=move_distance).destination(point=g_Point(self.stuck_lat, self.stuck_lon), bearing=(self.stuck_yaw+90*self.stuck_direction)%360) #first time, move to right
					self.target_lat,self.target_lon = target.latitude, target.longitude
					if len(self.target_lat_list) > self.stuck_nstop: #meaning it didn't reach the 5m to left/right; the success block is not executed
						self.target_lat_list[0]=self.target_lat #change the new stop; don't add new stop  
						self.target_lon_list[0]=self.target_lon 
					else: #meaning it reach the 5m to left/right; the success block is executed
						self.target_lat_list.insert(0,self.target_lat) #add a new stop
						self.target_lon_list.insert(0,self.target_lon)
					
				self.drive = -1.0; self.turn =0 #make it backward for three seconds
				time.sleep(3)
			else:
				self.stuck = 0 
				self.stuck_itick = self.itick #to ensure next check 10s later 
		
		if self.enabled == False: 
			self.distances = deque([],maxlen=500)
			self.stuck_itick = self.itick   #to ensure next stuck check 10s later
		
		if (self.success) and len(self.target_lat_list) ==0: #reach final stop
			print('!!!! COMPLETE !!!!')
			self.enabled = False

		if (self.success) and len(self.target_lat_list) > 0 and self.current_lon!=0 and self.enabled:
			print('!!!! reach one stop; now move to next')
			#self.itick = 0 # this itick will be used to determine the maximum turn 
			self.success = False
			self.target_lat = self.target_lat_list.pop(0)
			self.target_lon = self.target_lon_list.pop(0)
			#self.drive = -1.0; self.turn =0 #make it backward for three seconds
			self.itick2 = 0 #itck2 for the itick after hitting a stop; used to allow zero drive and full turn when the error_yaw is large (e.g., 180 degree turn) 
			time.sleep(1) #inneria movement by another 3 second; need to reduce this sleep time when bot move fast
		
		self.write_data()
	


	# For giving motion control to ESC; only called when auto PID control is enabled; called in the Simutaneous.py
	def getMotion(self):
		if (self.has_start_coords and (not self.success)):
			return [self.clampOutput(self.drive), self.clampOutput(self.turn)]
		return [0.0, 0.0] #if success or has no GPS reading

	def get_yaw_offset_with_gps(self):
		# use gps yaw reading to correct amg yaw reading to include 1) impact of background current or wind drag; 2)internal error in amg yaw reading
		# change offset if 1) straight line (rvalue>0.95) 2) small amg_yaw and gps_yaws' std 3) 10s after previous correction; 4) no duplicate lon/lat; 5) full speed in PID.drives or move fast (e.g., on land)
		ntime = 8                 #correction interval in seconds; larger value to get the rvalue for a longer period
		update_interval = 8       #update interval; 5 means, update gps yaw correction every 5 seconds; recommend using a larger value than ntime
		min_distance = ntime*0.5   #minimum distance over the ntime period; the small motor has speed of 0.67 m/s. 
		min_rvalue = 0.95          #minimum absolute rvalue for the robot track over the ntime period 
		tps = int(1/self.dt)       #tick per second; if dt=0.05, tps is 20 
		if self.enabled == False or len(self.distances)<tps*ntime: return #must be enabled 

		lons = np.array(self.current_lons)[0:ntime*tps]
		lats = np.array(self.current_lats)[0:ntime*tps]
		distances = np.array(self.distances)[0:ntime*tps] #not used for now
		fp = np.array([lat in [0.0,-1.0] for lat in lats]) #flag for invalid lat value
		if len(np.unique(lats))==1:  #not moving at all; or gps reading stuck; or all in 0 or -1. 
			self.gps_yaw_rvalue = -1 #all lats are same 
			return 
			
		if sum(fp)>0: #there is invalid value in lon/lat, with lat==0.0 or lat==-1.0
			self.gps_yaw_rvalue = -2  
			return 

		# get second-avg lons, lats, distances
		lons = lons.reshape([ntime,tps]).mean(axis=1) #average every second
		lats = lats.reshape([ntime,tps]).mean(axis=1) #average every second
		distances = distances.reshape([ntime,tps]).mean(axis=1) #avg every second; not used for now

		# don't do correction when close to destination
		if np.min(distances) < self.success_distance + 3: #0.5m ahead before reaching the destination
			self.gps_yaw_rvalue = -3
			return
		
		try: 
			rvalue = abs(linregress(lons,lats).rvalue) #r-value for lons and lats !! need to find a solution if moving eastward, westard, northward, or southward
		except: #have problem to do the linear regression, likely because no change of lon/lat
			self.gps_yaw_rvalue = -5
			return
		
		self.gps_yaw_rvalue = rvalue
		# get mean and std of gps and current yaws 
		gyaws = [] #get GPS yaws; do not use the yaw from first to final; as in some case, the vessle may reverse back (still straight line)
		for ii in range(len(lons)-1):
			gyaws.append(calculate_yaw(lons[ii+1], lats[ii+1], lons[ii],lats[ii])) #get direction from (lon1,lat1) to (lon2,lat2)
		
		cyaws = [mean_yaw(ii) for ii in np.array(self.current_yaws)[0:ntime*tps].reshape([ntime,tps])] #get second-avg current yaws
		
		self.mean_cyaw=mean_yaw(cyaws)
		self.mean_gyaw=mean_yaw(gyaws)
		self.std_cyaw=std_yaw(cyaws)
		self.std_gyaw=std_yaw(gyaws)
		self.distance5 = g_geodesic((lats[0], lons[0]), (lats[-1], lons[-1])).meters
		
		if abs(self.itick - self.itick_gps_correction) <tps*update_interval: return  #while the the number are calculated every tick; only update in every few second, avoid too frequent update

		# when std of current_yaw and gps_yaw are within 5 and 2; it means straight
		if self.yaw_correction_with_gps and self.distance5 > min_distance and self.std_cyaw < 5 and self.std_gyaw < 5:    #both stable with std within 25 degrees
			self.yaw_offset_from_gps += diff_yaw(mean_yaw(cyaws), mean_yaw(gyaws)) #on top of previous offset; use += instead of =; because current_yaw has amg_yaw + previous gps_offset
			self.yaw_offset_from_gps = (self.yaw_offset_from_gps+180)%360-180      #from [0,360] to [-180,180]
			self.yaw_offset_from_gps = np.clip(self.yaw_offset_from_gps,-90,90)   #set the maximum correction
			self.itick_gps_correction = self.itick
	
	# Second method of PID
	def tick2(self):
		self.itick += 1 
		self.itick2 += 1 #tick after reaching a stop point

		self.get_yll() #get error yaw, error_distance, target_distance, target_yaw; lon,lat and yaw are updated in Sim.py

		if self.yaw_source=='amg': self.get_yaw_offset_with_gps()  #regardless of turn it on or not; note that the yaws have frequency of 1s instead 0.05s
		                                #during the turn, we don't calculate the gps yaw offset
		# if error_yaw == None, due to either not moving or length of gps records
		# move the robot forward by 1 second
		if self.error_yaw==None:
			self.turn, self.drive = 0, 1
			time.sleep(1)
			self.write_data()
			return

		if abs(self.error_yaw)<20:   #using 0.1 turn; put this number to 180 to using 0.1 turn only
			turn_class = 10
		elif abs(self.error_yaw)>60: #using 0.3 turn
			turn_class = 30
		else:                        #using 0.2 turn 
			turn_class = 20          
		self.turn_class = turn_class

		self.ttime = abs(self.error_yaw) / self.dps[turn_class]  #update turn time using updated dps
		
		start_yaw = self.current_yaw
		start_error_yaw = self.error_yaw
		if abs(self.ttime) > 0.5: #when turn time is larger than 0.5 seconds
			self.turn = np.sign(self.error_yaw)*turn_class/100 #based on error_yaw and turn class		
			self.drive = 1

			start_time = time.perf_counter() #used to find the time elaps for the turn
			for ii in range(int(self.ttime/0.05)):
				self.get_yll()  #get error yaw, error_distance, target_distance, target_yaw; lon,lat and yaw are updated in Sim.py
				                #if without this, current yaw is still updatting, but not error_yaw

				# when error yaw is within dps/2 or within success distance or error_yaw switch sign (to avoid overshooting)
				if (abs(self.error_yaw) <= self.dps[turn_class]/2) or (self.error_distance < self.success_distance) or (abs(self.error_yaw) <= self.dps[turn_class] and np.sign(self.error_yaw)*np.sign(start_error_yaw)==-1): # if dps is 10, meaning error within 5
					break

				if ii%20==0: #every 1 second, do following
					self.write_data()

				time.sleep(0.05) 

			#update the dps when turn time larger than 2 seconds
			if self.ttime>2: 
				end_yaw = self.current_yaw
				end_time = time.perf_counter()
				tmp = abs(diff_yaw(start_yaw, end_yaw))/(end_time - start_time) #the actual turn during the this turn itme
				self.dps[turn_class] = np.clip(tmp,self.dps_min[turn_class],self.dps_max[turn_class])
				
		self.ttime = 0 #resume the ttime to 0; for data writing purpose
		
		# full speed forward over pid2_interval seconds
		self.turn, self.drive = 0, 1
		
		# move straightforward for longer time when the error yaw is within 5 degree
		if abs(self.error_yaw) <= 5 and self.error_distance > 20:
			stime = 1 #need a longer time for gps correction
		else:
			stime = self.pid2_interval
			
		for ii in range(int(stime/0.05)):
			time.sleep(0.05)
			self.get_yll()
			if ii%20 == 0: #write data every 1 second
				self.write_data()
		
		# Success condition
		self.success = (self.error_distance < self.success_distance)

		if self.enabled == False: 
			self.distances = deque([],maxlen=500)
			self.stuck_itick = self.itick   #to ensure next stuck check 10s later
		
		if (self.success) and len(self.target_lat_list) ==0: #reach final stop
			print('!!!! COMPLETE !!!!')
			self.enabled = False

		if (self.success) and len(self.target_lat_list) > 0 and self.current_lon!=0 and self.enabled:
			print('!!!! Reach one stop; now move to next') 
			time.sleep(3) #inneria movement by another 3 second; need to reduce this sleep time when bot move fast
			
			# lol,lat for next stop
			self.success = False
			self.target_lat = self.target_lat_list.pop(0) #get lat of next stop and remove the lon from the list 
			self.target_lon = self.target_lon_list.pop(0)

			# use 100% of turn after hitting the stop
			self.target_yaw = self.compute_target_yaw()
			self.error_yaw = self.compute_error_yaw()
			self.turn = 1*np.sign(self.error_yaw); self.drive = 0 
			for ii in range(int(abs(self.error_yaw)/self.dps2/0.05)):
				time.sleep(0.05)
				self.get_yll()
				if ii%20 == 0: #write data every 1 second
					self.write_data()

				if (abs(self.error_yaw) <= self.dps2/2): # if dps2 is 60, meaning error within 30
					break

			self.itick2 = 0 #itck2 for the itick after hitting a stop; used to allow zero drive and full turn when the error_yaw is large (e.g., 180 degree turn) 
		
		self.write_data()
