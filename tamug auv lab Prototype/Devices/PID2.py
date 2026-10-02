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

class PID2:
	
	def __init__(self, movement_distance=10, movement_direction=-90, drive_gains=[0.2, 0.0, 0.0], turn_gains=[0.01, 0.0, 0.02], dt=1,
	            fname_coords='../Data/target_coords.csv', success_distance=5,turn_lim=[0.2,0.4],max_turn_factor=0.01):
		
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
		self.error_distance_derivative = 0.0
		self.current_yaw = 0.0
		
		# Target Distance and Yaw
		self.target_distance = 0.0
		self.target_yaw = 0.0
		
		# Errors in Distance and Yaw
		self.error_distance = 0.0
		self.error_yaw = 0.0
		self.error_yaw_derivative = 0.0
		
		# Previous Errors in Distance and Yaw (Used for Derivative estimate)
		self.previous_error_distance = 0.0
		self.previous_error_yaw = 0.0
		self.previous_error_yaw2 = 0.0
		
		# PID control
		self.has_start_coords = False
		self.drive = 0.0
		self.turn = 0.0
		self.success = False #False until get the final stop
		self.success_distance=success_distance
		
		# PID gains for drive output
		self.Kp_drive = drive_gains[0]
		self.Ki_drive = drive_gains[1]
		self.Kd_drive = drive_gains[2]
		
		# PID gains for turn output
		self.Kp_turn = turn_gains[0]
		self.Ki_turn = turn_gains[1]
		self.Kd_turn = turn_gains[2]
		
		# Minimum and Maximum output
		self.MIN_OUTPUT = -1.0
		self.MAX_OUTPUT = 1.0
		self.max_turn = 0.5
		self.turn_lim = turn_lim
		self.max_turn_factor = max_turn_factor

		self.itick = 0
		self.turns = deque([],maxlen=500)
		self.drives = deque([],maxlen=500)
		self.current_yaws = deque([],maxlen=500)
		self.error_yaws = deque([],maxlen=500)
		self.current_lons = deque([],maxlen=500)
		self.current_lats = deque([],maxlen=500)
		self.distances = deque([],maxlen=500)
		self.stuck = 0
		self.stuck_lon = 0
		self.stuck_lat = 0
		self.stuck_yaw = 0 
		self.stuck_itick = 0
		self.stuck_distance = 0 
		self.stuck_direction = 0 #what direction to move when stuck
		self.stuck_nstop = len(self.target_lat_list)
		self.enabled = False

		# for writing 
		self.fid = None
		self.writing_interval = 1 # writing interval in seconds; use 0 for writing in every time step
		self.previous_writing_time = datetime.now()
		self.writing_dir = '../Data/PIDlogs/'
		if os.path.exists(self.writing_dir): os.makedirs(self.writing_dir,exist_ok=True)
	
	def printData(self):
		
		print(f'\n=== PID2 (enabled:{self.enabled}): {len(self.target_lat_list)+1} STOPS remaining ===')
		print(f'target_lat: {self.target_lat:.6f}; target_lon: {self.target_lon:.6f}')
		print(f'distance: {self.error_distance:.1f} (m)  | drive: {self.drive:.1f}')
		print(f'current_yaw: {self.current_yaw:.2f} | target_yaw: {self.target_yaw:.2f}')
		print(f'error_yaw: {self.error_yaw:.2f}    | error_yaw_derivative: {self.error_yaw_derivative:.2f} | turn: {self.turn:.2f}')
		print(f'previous_yaw_error: {self.previous_error_yaw:.2f}')
		print(f'length of yaws {len(self.error_yaws)}')
		#print(f'delta yaw_error: {self.error_yaw-self.previous_error_yaw2:.02f}')
		print(f'K_turn:{self.Kp_turn:.2f} | D_turn:{self.Kd_turn:.2f}')

	def write_data(self):
		if self.enabled:
			if self.fid==None or self.fid.closed:
				timestamp = datetime.now().strftime('%Y%m%d%H%M')
				fname=f'{self.writing_dir}/PIDlogs_{timestamp}.csv'
				print(f'Opening {fname}')
				self.fid = open(fname,'w')
				self.fid.write('time,stops,lat,lon,distance,drive,target_yaw,current_yaw,error_yaw,error_yaw_derivative,turn,max_turn,stuck\n')
			# write information to csv file
			if (datetime.now() - self.previous_writing_time).seconds >= self.writing_interval:
				timestamp = datetime.now().strftime('%Y%m%d%H%M%S')
				line=f'{timestamp},{len(self.target_lon_list)+1},{self.current_lat:.6f},{self.current_lon:.6f},{self.current_distance:.2f},{self.drive:.2f},'\
					+f'{self.target_yaw:.1f},{self.current_yaw},{self.error_yaw},{self.error_yaw_derivative},{self.turn:.2f},{self.max_turn:.2f},{self.stuck}\n'
				self.fid.write(line)
				self.previous_writing_time = datetime.now()
		else:
			if self.fid != None and not self.fid.closed: self.fid.close()

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
		if self.current_yaw==None: return 0.0
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
	
	# Take yaw from GPS
	def takeYaw(self, yaw):
		if yaw is None:  #likely because of no movement; make it move forward for three seconds
			self.current_yaw = self.target_yaw
		else:
			self.current_yaw = yaw % 360  # Ensure yaw is in [0, 360) range
	
	# Tick the PID with latest response variables
	def tick(self):
		self.itick += 1
		# Get target yaw
		self.target_yaw = self.compute_target_yaw()
		
		# Errors
		self.error_distance = self.current_distance
		self.distances.appendleft(self.error_distance)

		self.error_yaw = self.compute_error_yaw()
		if self.error_yaw==None: #when GPS give None yaw; move forward for pid.dt second until GPS give another valid reading
			self.drive = 1
			self.turn = 0
			time.sleep(0.5)
			self.error_yaws.appendleft(None)  #need to add this None, needed when calculating error derivative; otherwise error derivative will use previous valid error_yaw (maybe from multiple second earlier)
			self.write_data()
			return
		
		# if there is valid GPS yaw reading
		self.error_yaws.appendleft(self.error_yaw)
		
		# Control (P and D components)
		# Drive control
		if len(self.distances)>=2: self.error_distance_derivative = (self.distances[0] - self.distances[1]) / self.dt
		self.drive = (self.Kp_drive * self.error_distance + 
					 self.Kd_drive * self.error_distance_derivative) 
		self.drive = np.clip(self.drive, -1, 1)

		# Turn control
		if len(self.error_yaws)>=2 and self.error_yaws[1]!=None: 
			self.error_yaw_derivative = (self.error_yaws[0] - self.error_yaws[1]) / self.dt
		else:
			self.error_yaw_derivative = 0.0
		self.turn = (self.Kp_turn * self.error_yaw + 
					self.Kd_turn * self.error_yaw_derivative)  # positive to turn left; negative to turn right
        
		# Limit the max turn; max_turn now based on abs(error_yaw)/100, with min and max specified in turn_lim
		self.max_turn = np.clip(abs(self.error_yaw)*self.max_turn_factor,self.turn_lim[0],self.turn_lim[1]) 
		self.turn = np.clip(self.turn, -self.max_turn, self.max_turn)
		
		self.drives.appendleft(self.drive)
		self.turns.appendleft(self.turn)
		
		# Previous errors; needed for printing; could be removed after testing 
		if len(self.distances)>=2: self.previous_error_distance = self.distances[1]
		if len(self.error_yaws)>=2: self.previous_error_yaw = self.error_yaws[1]
		
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
				if abs(self.error_distance-move_distance)<1: 
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
		
		if (self.success) and len(self.target_lat_list) > 0 and self.current_lon!=0 and self.enabled:
			print('!!!! REACH ONE STOP; now move to next')
			self.itick = 0 
			self.success = False
			self.target_lat = self.target_lat_list.pop(0)
			self.target_lon = self.target_lon_list.pop(0)
			#self.drive = -1.0; self.turn =0 #make it backward for three seconds
			time.sleep(1)
		
		self.current_lons.appendleft(self.current_lon)
		self.current_lats.appendleft(self.current_lat)
		self.current_yaws.appendleft(self.current_yaw)
		
		self.write_data()
	
	# For giving motion control to ESC; only called when PID control is enabled
	def getMotion(self):
		if (self.has_start_coords and (not self.success)):
			return [self.drive,self.turn]
			#return [self.clampOutput(self.drive), self.clampOutput(self.turn)]
		return [0.0, 0.0] #if success or has no GPS reading


