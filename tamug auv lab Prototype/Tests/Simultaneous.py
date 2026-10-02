import os
import sys
import math
import time
import threading
from datetime import datetime
from inputs import devices, get_key
import numpy as np
from functions import diff_yaw,mean_yaw,std_yaw,calculate_yaw
from collections import deque
#==================== SETTINGS ============================
debug            = 0        # debug for shorter run time and keep all screen outputs1
get_dps          = 0        # to get DPS (degree turned per second)
take_fake_gps    = 1        # take a fake gps if gps has no fix; if gps has fix, it will take gps reading

# for ESC
speeding         = 1.0      # in range of [0.5, 1.0]
turn_offset      = -0.00  # the background value needed to make the robot move straight forward 

# for PID
dt_pid           = 0.05
drive_gains      = [0.05, 0.0, 0.0]  #0.2 for model 3; 0.05 for model 4, half power when within 20m
turn_gains       = [0.02, 0.0, 0.00] #0.01 for model 3; 0.02 for model 4
delta_turn       = 0.02             #limit dramatic turn change, maximum change of turn in pid_dt, 0.02 will lead to maximum 0.4 change of turn
delta_drive      = 0.01             #limit dramatic change of drive to avoid crazy nose up at the very beginning 
fname_coords     = '../Data/target_coords_2stops.csv'  #lat/lon of stops
#fname_coords     = '../Data/target_coords_2stops.csv'    #lat/lon of stops
pid_option       = 1        #1 for caculating turn based on error yaw; 2 for calculating turn TIME based on error yaw 
yaw_source       = 'amg'    #or 'amg', the source sensor for current yaw reading
success_distance = 5        #in meters; within this distance, robot will go to next stop
yaw_correction_with_gps = 1  #used used when yaw_source == 'amg'

# for AMG; used only when yaw_source == 'amg'
offset           = -75        #this offset ise used to correct reading from AMG yaw reading; find this number by facing vessle to first destination and use the error_yaw
manual_correction= 0          # interp amg yaw based on '../Data/manual_correction.csv'
ntick_avg        = 1

#===================== create object for each module =================
devices = ['PID','ESC'] #,'GPS','ESC','AMG','XCON']
sys.path.append(os.path.abspath('../Devices/'))
if 'GPS' in devices:
	import GPS
	gps=GPS.GPS()

import ESC
esc=ESC.ESC([19,20], speeding=np.clip(speeding,0.5,1.0),turn_offset=turn_offset)

if 'AMG' in devices:
	import AMG
	amg=AMG.AMG(offset=offset, manual_correction=manual_correction,ntick_avg=ntick_avg)

if 'XCON' in devices:
	import XCON
	xcon=XCON.XCON()

# PID will use amg and gps
import PID
pid=PID.PID(success_distance=success_distance, fname_coords=fname_coords, 
			yaw_correction_with_gps=yaw_correction_with_gps,yaw_source=yaw_source,
			drive_gains=drive_gains, turn_gains=turn_gains, dt=dt_pid, delta_turn=delta_turn,delta_drive=delta_drive)

#=====================================================
running = True
pidControlEnabled = False #False
breakMain = False
keyControl = 'None'
keyStates = {
	'KEY_UP': False,
	'KEY_DOWN': False,
	'KEY_LEFT': False,
	'KEY_RIGHT': False,
	'KEY_S': False,  #start PID (autonomous control)
	'KEY_E': False,  #end PID
	'KEY_K': False,  #kill, not used for now
	'KEY_P': False,  #plus, get_dps +1; not used for now
	'KEY_M': False,  #minus, get_dps -1; not used for now
	'KEY_EQUAL': False, #plus turn offset, make it moving left
	'KEY_MINUS': False,  #minus turn offset, make it moving right
	'KEY_R': False, #restart the PID, resume the original stop lists
	'KEY_G': False,  #start GPS writing
	'KEY_T': False   #terminate GPS writing
}

def loopESC():
	global pidControlEnabled, breakMain,get_dps
	esc.arm()
	esc.setMotionModel('Differential')
	while running:
		##Either XBOX controller or keyboard
		#print(keyControl)
		if (keyControl == 'None'):  #no keyboard input 
			if 'XCON' not in devices:
				if pidControlEnabled:
					esc.setMotion(pid.getMotion())
				else:
					esc.setMotion([0.0, 0.0])
			else:
				#if xcon.LB and xcon.RB and xcon.DS:  #right-bumper + left-bumper + down
				#	breakMain = True
				if xcon.LB:
					pidControlEnabled = True
					pid.enabled = True
					pid.itick = 0
				if xcon.RB:
					pidControlEnabled = False
					pid.enabled = False
				if pidControlEnabled: #without any xcon input
					esc.setMotion(pid.getMotion())
				else:
					#if using left/right stick, make sure to disable the PID control (by press right bumper) first 
					esc.setOne(np.clip(np.round(-xcon.LY *10) * 10,-100,100), 2) #left stick for second motor
					esc.setOne(np.clip(np.round(xcon.RY * 10) * 10,-100,100), 1)  #right stick for first motor 
				if xcon.DN:   #north
					esc.speeding += 0.1
					esc.speeding = max([0.0, min([1.0,esc.speeding])]) #keep minimum of 0.5+
					time.sleep (1)  #sleep 1 second to ensure decreasing speeding by 0.1 everytime
				if xcon.DS:  #south
					esc.speeding -= 0.1
					esc.speeding = max([0.0, min([1.0,esc.speeding])]) #keep minimum of 0.5
					time.sleep (1)
		##background turn 
		elif (keyControl == 'KEY_EQUAL'):
			esc.turn_offset -= 0.01  #make the robot to veering right
			time.sleep(0.5)
		##Slow down incrementally
		elif (keyControl == 'KEY_MINUS'):
			esc.turn_offset += 0.01  #make the robot to veering left
			time.sleep(1)
		##Start autonomous control
		elif (keyControl == 'KEY_S'):
			pidControlEnabled = True
			pid.enabled = True
			pid.itick = 0
		##End autonomous control
		elif (keyControl == 'KEY_E'):
			pidControlEnabled = False
			pid.enabled = False
			pid.itick = 0
		elif (keyControl == 'KEY_G'):
			gps.write_data = True
		elif (keyControl == 'KEY_T'):
			gps.write_data = False
		elif (keyControl == 'KEY_R'): #reset the autonomous control
			pidControlEnabled = False
			pid.enabled = False
			pid.itick = 0 
			# reset the lon/lat list
			tmp=np.array([line.strip().split(',')  for line in open(fname_coords, 'r').readlines()]).astype('float')
			pid.target_lat_list = list(tmp[:,0]) #must be a list
			pid.target_lon_list = list(tmp[:,1])
			pid.target_lat = pid.target_lat_list.pop(0)
			pid.target_lon = pid.target_lon_list.pop(0)
			#pass
		##Ends program before RUNTIME ends
		#elif (keyControl == 'KEY_K'):
		#	breakMain = True
		##Turns off keyboard controls and sets motion to max in a specific direction
		elif (keyControl == 'KEY_UP'):
			esc.setMotion([1, 0.0])
			pidControlEnabled = False
			pid.enabled = False
		elif (keyControl == 'KEY_DOWN'):
			esc.setMotion([-1, 0.0])
			#time.sleep(3)
			pidControlEnabled = False
			pid.enabled = False
		elif (keyControl == 'KEY_LEFT'):
			# when trying to get dps (degree turn per second), it will use 10% of turn on left key 
			print(get_dps)
			if get_dps==1: 
				esc.setMotion([1.0, 0.1])  #turn positive, like the error_yaw is positive
			elif get_dps==2:
				esc.setMotion([1.0, 0.2])
			elif get_dps==3:
				esc.setMotion([1.0, 0.3])
			else:
				esc.setMotion([0.0, 1.0])  #turn positive, like the error_yaw is positive
			pidControlEnabled = False
			pid.enabled = False
		elif (keyControl == 'KEY_RIGHT'):
			if get_dps==1: 
				esc.setMotion([1.0, -0.1])
			elif get_dps==2:
				esc.setMotion([1.0, -0.2])
			elif get_dps==3:
				esc.setMotion([1.0, -0.3])
			else:
				esc.setMotion([0.0, -1.0])
			pidControlEnabled = False
			pid.enabled = False
		elif (keyControl == 'KEY_P'):
			#get_dps = np.clip(1+get_dps,0,3)
			time.sleep(0.5) 
		elif (keyControl == 'KEY_M'):
			#get_dps = np.clip(get_dps-1,0,3) 
			time.sleep(0.5)
		
		time.sleep(0.05)
	esc.disarm()

def loopTDS():
	while running:
		tds.tick()
		time.sleep(0.05)

def loopAMG():
	while running:
		amg.tick()
		if yaw_source == 'amg': pid.takeYaw(amg.getYaw()) #getYaw() will give amg.yaw + amg.offset
		time.sleep(0.05)

def loopGPS():
	while running:
		gps.tick()
		if yaw_source == 'gps': pid.takeYaw(gps.getYaw(option=2,ntime=5,max_std=2))
		if gps.hasFix:
			pid.takeCoords(gps.latitude, gps.longitude)
		elif take_fake_gps: 
			pid.takeCoords(29.31364698410316, -94.8170861900211) #pretending it has a gps reading, needed for PID control
		time.sleep(0.01)

def loopPID():
	while running:
		if pid_option==1:
			pid.tick()
		elif pid_option==2:
			pid.tick2()
		time.sleep(pid.dt)

def loopKEYBOARD():
	global keyControl,pidControlEnabled
	global keyStates
	codes = deque([],maxlen=2)
	try:
		while running:
			events = get_key()
			keyControl = 'None'
			for event in events:
				if event.ev_type == 'Key':
					codes.appendleft(event.code)
					#time.sleep(0.5)
					#print(event.code)
					#print(list(codes))
					if list(codes) == ['KEY_S', 'KEY_LEFTALT']: 
						#pidControlEnabled = True
						#pid.enabled = True
						pid.itick = 0
					if list(codes) == ['KEY_E', 'KEY_LEFTALT']:
						#pidControlEnabled = False
						#pid.enabled = False
						pid.itick = 0
					if event.code in keyStates:
						keyStates[event.code] = (not (event.state == 0))
			for key, held in keyStates.items():
				if held:
					keyControl = key
	except Exception as e:
		print(e)

def loopXCON():
	xcon.start()
	while running:
		time.sleep(0.05)
	try:
		xcon.stop()
	except:
		pass

def loopMain():  #for printing data
	global use_gps_yaw
	previous_itick = 0
	global running #need to use global in order for change of this variable to be felt by other threads
	runtime = 10800 #in seconds
	fps = 1        #frequency per second
	if debug:
		runtime=20
		fps=1
	for i in range(runtime*fps):
		if not debug: os.system('clear')
		for module in devices:
			exec(f'{module.lower()}.printData()')
		
		print(f'\n{i+1}/{runtime*fps}; get_dps:{get_dps}')
		if debug: print(f'itick for GPS,PID {gps.itick},{pid.itick}')

		if breakMain:
			break
		
		time.sleep(1/fps)

	running = False
	

def thr(name):
	return threading.Thread(target = name, args = ())

# create multiple threads for parallel ticking of all devices
threads = [thr(loopMain)]
for module in devices:
	exec(f'threads.append(thr(loop{module}))')
threads.append(thr(loopKEYBOARD))

for t in threads:
	t.start()

# threads will end only when running==False (controlled in loopMain)	
for t in threads:
	t.join()

