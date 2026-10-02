import os
import time
import math
import numpy as np
import board
import busio
import adafruit_lsm9ds1

S1 = adafruit_lsm9ds1.LSM9DS1_I2C(busio.I2C(board.SCL, board.SDA))

GX = []
GY = []
GZ = []

N = 1000
for i in range(N):
	gx, gy, gz = S1.gyro
	GX.append(gx)
	GY.append(gy)
	GZ.append(gz)
	print('gx: ',gx)
	print('gy: ',gy)
	print('gz: ',gz)
	print(f'{i}/{N}')
	time.sleep(0.05)
	os.system('clear')

goff = np.array([np.mean(GX),np.mean(GY),np.mean(GZ)])

print('Avg gx: ',goff[0])
print('Avg gy: ',goff[1])
print('Avg gz: ',goff[2])

fid = open('GyroCalibration.txt','w')
fid.write(str(goff[0])+'\n')
fid.write(str(goff[1])+'\n')
fid.write(str(goff[2])+'\n')
fid.close()
