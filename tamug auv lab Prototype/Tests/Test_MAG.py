# https://www.desmos.com/3d/2qr0bjv50s
# Red tape marker calibration (inside)

import smbus2
import time
import math
import os
from RPLCD.i2c import CharLCD

#lcd = CharLCD('PCF8574', 0x27)

bus = smbus2.SMBus(1)
BMM150 = 0x13

# Power Changer |||| (0x4B)
# "On" ------------- (0x01)
# "Off" ------------ (0x00)
bus.write_byte_data(BMM150, 0x4B, 0x01)
time.sleep(0.01)

# OpMode Changer ||| (0x4C)
# "Normal" --------- (0x00)
# "High accuracy" -- (0x08)
# "Sleep" ---------- (0x11)
bus.write_byte_data(BMM150, 0x4C, 0x08)
time.sleep(0.01)

def tc(val, bits):
    if val & (1 << (bits - 1)):
        val -= 1 << bits
    return val

def read_data():
	
    data = bus.read_i2c_block_data(BMM150, 0x42, 8)
    
    x = ((data[1] << 5) | (data[0] >> 3)) & 0x1FFF
    y = ((data[3] << 5) | (data[2] >> 3)) & 0x1FFF
    z = ((data[5] << 7) | (data[4] >> 1)) & 0x7FFF

    x = round(tc(x, 13)*0.3,1)
    y = round(tc(y, 13)*0.3,1)
    z = round(tc(z, 15)*0.3,1)

    return x, y, z

def hard_iron_fix(x, y, z):
	xfix = x - (9.5960)
	yfix = y - (-5.5325)
	zfix = z - (7.6735)
	return xfix, yfix, zfix

file_magData = open('../Data/RawMagReadings.txt','w')

file_magData.write('[')
#print('[')

for i in range(300):
	
	if i != 0:
		file_magData.write(',')
	
	x, y, z = read_data()
	
	xs, ys, zs = '+','+','+'
	if x < 0: xs = ''
	if y < 0: ys = ''
	if z < 0: zs = ''
	
	xf, yf, zf = hard_iron_fix(x, y, z)
	
	offsetAngle = 0.0312350616312 # rad
	
	A = math.degrees(math.atan2(yf,xf) - offsetAngle)
	
	os.system('clear')
	print(f'X:{xf:.4f} Y:{yf:.4f} Z:{zf:.4f}\nA:{A:.1f}')
	
	#lcd.clear()
	#lcd.write_string(f'X:{xs}{x:.1f} Y:{ys}{y:.1f}')
	#lcd.cursor_pos = (1, 0)
	#lcd.write_string(f'Z:{zs}{z:.1f} A:{A:.1f}')
	
	file_magData.write(f'({x},{y},{z})')
	
	time.sleep(0.5)

file_magData.write(']')
#print(']')

file_magData.close()
