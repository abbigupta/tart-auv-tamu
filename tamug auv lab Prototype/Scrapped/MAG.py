import math
import smbus2
import time
from RPLCD.i2c import CharLCD

def tc(val, bits):
	if val & (1 << (bits - 1)):
		val -= 1 << bits
	return val

def calibrate(x, y, z):
	x -= (9.53564614)
	y -= (-5.15776442)
	z -= (0.37741725)
	x = (0.97641)*x + (-0.0026403)*y + (0.00060812)*z
	y = (-0.0023486)*x + (0.96525)*y + (-0.0020202)*z
	z = (-0.0012332)*x + (-0.0038999)*y + (1.0561)*z
	return x, y, z

def pitchBack(x, y, z, pitch):
	x = x*math.cos(-pitch) + z*math.sin(-pitch)
	y = y
	z = -x*math.sin(-pitch) + z*math.cos(-pitch)
	return x, y, z

def rollBack(x, y, z, roll):
	x = x
	y = y*math.cos(roll) + z*math.sin(roll)
	z = -y*math.sin(roll) + z*math.cos(roll)
	return x, y, z

class MAG:
	
	lcd = 'null'
	
	x = 0.0
	y = 0.0
	z = 0.0
	
	xc = 0.0
	yc = 0.0
	zc = 0.0
	
	magnitude = 0.0
	yaw = 0.0
	
	filterWeight = 0.0
	
	def __init__(self, address = 0x13, bus = smbus2.SMBus(1)):
		self.bus = bus
		self.address = address
		self.bus.write_byte_data(address, 0x4B, 0x01)
		time.sleep(0.01)
		self.bus.write_byte_data(address, 0x4C, 0x08)
		time.sleep(0.01)
	
	def printRawData(self):
		print(f'MagX: {self.xc}')
		print(f'MagY: {self.yc}')
		print(f'MagZ: {self.zc}')
	
	def printDataLCD(self):
		if (self.lcd == 'null'):
			self.lcd = CharLCD('PCF8574', 0x27)
		xs = '+'
		ys = '+'
		zs = '+'
		if (self.xc < 0): xs = ''
		if (self.yc < 0): ys = ''
		if (self.zc < 0): zs = ''
		self.lcd.clear()
		self.lcd.write_string(f'X:{xs}{self.xc:.1f} Y:{ys}{self.yc:.1f}')
		self.lcd.cursor_pos = (1, 0)
		self.lcd.write_string(f'Z:{zs}{self.zc:.1f} A:{math.degrees(self.yaw):.1f}')
		
	
	def printData(self):
		print('\n---Magnetometer------------')
		#print(f'Magnitude: {self.magnitude:.2f}')
		print(f'Yaw: {math.degrees(self.yaw):.1f}')
	
	def getYaw(self):
		return self.yaw
	
	def tick(self, pitch, roll):
		
		data = self.bus.read_i2c_block_data(self.address, 0x42, 8)
		
		x = round(tc(((data[1] << 5) | (data[0] >> 3)) & 0x1FFF, 13)*0.3, 1)
		y = round(tc(((data[3] << 5) | (data[2] >> 3)) & 0x1FFF, 13)*0.3, 1)
		z = round(tc(((data[5] << 7) | (data[4] >> 1)) & 0x7FFF, 15)*0.3, 1)

		self.x = x
		self.y = y
		self.z = z
		
		xf, yf, zf = calibrate(x, y, z)
		xf, yf, zf = pitchBack(xf, yf, zf, pitch)
		xf, yf, zf = rollBack(xf, yf, zf, roll)
		
		self.xc = xf
		self.yc = yf
		self.zc = zf
		
		self.magnitude = math.sqrt(xf**2 + yf**2 + zf**2)
		
		self.yaw = self.filterWeight*self.yaw + (1.0-self.filterWeight)*math.atan2(yf,xf)
