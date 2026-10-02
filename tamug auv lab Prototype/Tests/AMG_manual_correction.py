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
sys.path.append(os.path.abspath('../Devices/'))
''' 
Why? Even with AMG calibraiton, readings from the AMG is sitll not perfect. When changing the robot
by 90 degrees, AMG reading some times give > 90 degree of change, while sometime giving <90 degree of change. 

The manual calibration is to try to create a curve to ensure when rotating 90 degree, the AMG reading also give 
90 degrees of change.

Steps: 
1. run the script
2. find the amg 0.0; for the first direction (i.e., 0.0), the mean yaw must be within [-1,1]
3. place a mark for the 0.0 direction
4. for the second and other directions, rotate the robot by the defined interval
'''
#============== SETTINGS =========
interval = 45     #interval of directioins
ntime    = 5      #duration for averaging AMG reading
max_std  = 2      #maximum standard deviation of AMG reading over the ntime seconds
#================================

import AMG
amg=AMG.AMG(offset=0, manual_correction=0)

running = True
def loopAMG():
	while running:
		amg.tick()
		time.sleep(0.05)
		
def loopMain():  #for printing data
	global running #need to use global in order for change of this variable to be felt by other threads
	ayaws=[]       #to store the averged amg yaw reading 
	while running:
		if len(amg.yaws)>20*ntime:
			for trueyaw in range(0,361,interval):
				#os.system('clear')
				print(f'For true direction {trueyaw:.2f} degree from AMG-0')
				input(f'After placing the robot to the right direction, press any key to confirm')
				time.sleep(2)
				
				while True:
					ayaw_mean = mean_yaw(np.array(amg.yaws)[0:20*ntime])
					ayaw_std = std_yaw(np.array(amg.yaws)[0:20*ntime])
					
					print(f'\n[AMG] current yaw: {amg.yaws[0]:.2f}; itick: {amg.itick}')
					print(f'[AMG] mean yaw: {ayaw_mean:.2f}; need to be witin [-1,1] for first direction (i.e., 0.0)')
					print(f'[AMG] std yaw: {ayaw_std:.2f}; need to be smaller than {max_std:.2f}')
					if trueyaw==0: #for the first direction (i.e., 0), need to make sure the ayaw is within 2 degree
						if abs(ayaw_mean)<1 and ayaw_std<max_std:
							ayaws.append(ayaw_mean%360)
							print(f'Accept value: {ayaw_mean%360:.1f} for true yaw {trueyaw:.0f}')
							time.sleep(3)
							break #going to next direction
					else:
						if ayaw_std<max_std:
							ayaws.append(ayaw_mean%360)
							print(f'Accept value: {ayaw_mean%360:.1f} for true yaw {trueyaw:.0f}')
							time.sleep(3)
							break #going to next direction
					time.sleep(1)
			
			# give warning if the last reading is not close to 0.0
			if abs(diff_yaw(ayaws[-1],0.0))<10:
				print('!!warning: the last amg yaw is ayaws[-1]; should be close to 0.0')
				print('!!if it is off too much from 0.0, it is better to rerun this script')
				ans=input('Do you want to save this result? (press y or Y)')
				if not ans in ['y','Y']: sys.exit('Terminated by user') 

			# write the data
			ayaws[0] = 0 #the first must be 0
			ayaws[-1] = 360 #the last must be 360
			data = np.column_stack((np.array(ayaws), np.arange(0,361,interval)))   # shape (n, 2)
			timestamp = datetime.now().strftime('%Y%m%d%H%M')
			fname=f'../Data/manual_correction_{timestamp}.csv'
			print(f'Data saved into file {fname}')
			np.savetxt(fname, data, delimiter=',', comments='', fmt='%.1f')

			# make symbolic link to the new manual_correction data 
			cmd=f'cd ../Data; ln -sf manual_correction_{timestamp}.csv manual_correction.csv'
			#os.system(cmd); print(cmd)

			# change running to False to terminate all threads
			running = False #after getting the AMG reading for all the directions
		time.sleep(1) # check 1 second later
	
def thr(name):
	return threading.Thread(target = name, args = ())

# create multiple threads for parallel ticking of all devices
threads = [thr(loopAMG),thr(loopMain)]

for t in threads:
	t.start()

# threads will end only when running==False (controlled in loopMain)	
for t in threads:
	t.join()
