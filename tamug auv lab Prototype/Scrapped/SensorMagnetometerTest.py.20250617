import smbus2
import time
import math
import os

bus = smbus2.SMBus(1)
BMM150_ADDRESS = 0x13
POWER_MODE_REGISTER = 0x4B
SOFT_RESET_COMMAND = 0x82

power_mode = bus.read_byte_data(BMM150_ADDRESS, POWER_MODE_REGISTER)
print(f"Power Mode Register: {power_mode:#04x}")

bus.write_byte_data(BMM150_ADDRESS, POWER_MODE_REGISTER, 0x01)
bus.write_byte_data(BMM150_ADDRESS, 0x4C, 0x08)

def read_data():
	data = bus.read_i2c_block_data(BMM150_ADDRESS, 0x42, 9)
	x = data[0] | (data[1] << 8)
	y = data[2] | (data[3] << 8)
	z = data[4] | (data[5] << 8)
	if x > 32767: x -= 65536
	if y > 32767: y -= 65536
	if z > 32767: z -= 65536
	#print(data)
	x*=0.3; y*=0.3; z*=0.3
	heading_deg = math.degrees(math.atan2(y,x))
	if heading_deg < 0: 
		heading_deg +=360
	return x, y, z,heading_deg

while True:
	os.system('clear')
	x, y, z, heading = read_data()
	print(f'X: {x:.2f} Y: {y:.2f}, Z: {z:.2f}, Heading: {heading:.0f}')
	time.sleep(0.05)
